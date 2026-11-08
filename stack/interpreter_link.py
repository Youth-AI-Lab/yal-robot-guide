"""The link from the Interpreter to the Strategist: an MCP server over HTTP, with a token. Not a tier: a door.

The Interpreter is a chatbot (Open WebUI, the voice of Feurisson, with its own model and its own instructions,
interpreter/feurisson.md). It talks with the group in the group's words, turns their request into a mission in the
closed format, has the group agree to it, then gives it to the Strategist through these tools. Nothing else of the
stack is reachable from it: never the robot, never the Pilot's tools.

  mission_options()            the closed lists: the steps, the cards the camera knows, the colours, the personalities
  start_mission(steps, personality)
                               call only after the group said yes to the reformulation: starts the mission
  pause_mission(), resume_mission(), stop_mission()
                               handled by the Strategist's loop and the Pilot without asking the model; the robot
                               stops on the next cell, as soon as possible (no adult needed)
  mission_status()             running, paused, done or stopped, and the report when it ends (closed fields)
And one route that is not a tool, for the filter of Open WebUI (interpreter/feurisson_filter.py), same token:
  POST /turn {"group_asked_to_stop": bool}
                               a new message of the group: answers mission_status, read now. From then on,
                               stop_mission is refused unless the group asked to stop, or the mission was paused in an
                               earlier turn (the group was asked): pause first, ask, then stop (rule 6 of
                               feurisson.md, which DeepSeek broke 4 times in 198 turns, whatever the instructions
                               said). Without the filter, nothing changes.
A mission that changes in nature is stopped, then the new one started; one that is only enriched too (the
Strategist starts a new conversation for each mission; the map of the terrain stays).

Three locks, as the kit's mcp_http.py: a bearer token (MCP_TOKEN, from the environment, never on the command line,
32 characters at least): a request without it gets 401; the Host header (DNS rebinding protection of the SDK): only
--allowed-host; and only these six tools. The Strategist's model and its key: --model, and the key on stdin (never on
the command line, never in a file).

Usage (the folder of the guide is the current folder; the simulator or the robot connected):
    MCP_TOKEN=... python3 -I stack/interpreter_link.py --model none --allowed-host 127.0.0.1:8790
    then, in Open WebUI: a tool server of type MCP (Streamable HTTP), http://<this computer>:8790/mcp, bearer token
"""
import argparse
import hmac
import json
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Literal

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from common import STATE_DIR  # noqa: E402
import strategist  # noqa: E402

CARDS = sorted(strategist.pilot.MAT["cards"])
SETUP = {"model": "none", "key": "", "tools": "route", "cap": strategist.RUN_CAP_USD}
RUN = {"thread": None, "id": None, "report": None, "error": None, "started": None, "task": None}
# The turns of the conversation, told by the filter: n counts the group's messages (0: no filter), paused_in is the
# turn of the last pause_mission, stop_asked whether the group's message asked to stop.
TURN = {"n": 0, "paused_in": None, "stop_asked": False, "read_in": None, "read_at": 0.0, "ordered": False}
# mission_status read again in the same answer while the mission runs: DeepSeek v4 Flash polled it right after
# start_mission until the end of the mission and said nothing to the group (5 turns in 198, 2026-11-08). Same answer:
# the same turn of the filter, or, without the filter, 10 seconds. Not after a pause, a stop or a resume of this turn:
# then waiting for "paused" or "stopped" is right.
READ_AGAIN = ("You read mission_status a moment ago in this same answer, and a mission takes minutes: tell the group "
              "now, in a few words, what it says. Read it again only when they ask.")
ASK_FIRST = ("you paused the mission in this same answer: ask the group first whether to change it, add to it, or let "
             "it go on, and call stop_mission only once they have answered")
PAUSE_FIRST = ("the group did not ask to stop: call pause_mission first, ask the group whether to change the mission, "
               "add to it, or let it go on, and call stop_mission only once they have answered")


def say(do):
    common.say_held(None)
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    (STATE_DIR / "control.json").write_text(json.dumps({"do": do}) + "\n", encoding="utf-8")


def state():
    if RUN["thread"] is None:
        return "no mission yet"
    if RUN["thread"].is_alive():
        if common.waiting_info():
            return "waiting for an adult"
        if common.mission_control() == "pause":
            return "paused" if common.held_where() else "pausing"
        return "running"
    if RUN["error"]:
        return "failed"
    return "stopped" if (RUN["report"] or {}).get("outcome") == "stopped" else "ended"


def mission_options() -> str:
    """The closed lists a mission is made of. Use them to turn what the group asks into a mission: a list of steps,
    each one an action with one argument from these lists, and a personality. Never invent a value outside them."""
    library = strategist.common.load_json(strategist.HERE / "personality" / "bricks.json")["bricks"]
    personalities = {}
    for path in sorted((strategist.HERE / "personality").glob("*.json")):
        if path.name != "bricks.json":
            bricks = strategist.common.load_json(path)["bricks"]
            personalities[path.stem] = [library[b].get("means") or library[b].get("text") for b in bricks]
    return json.dumps({
        "steps": {"reach_card": "go to a card and stop right in front of it, facing it (argument: card)",
                  "set_light": "set the robot's light (argument: color)",
                  "find_card": "find out whether a card is on the mat, and on which cell (argument: card)",
                  "survey": "find every card of one kind, with the cell of each (argument: card)"},
        "cards": {c: strategist.pilot.MAT["cards"][c] for c in CARDS},
        "colors": list(strategist.COLORS),
        "personalities": personalities,
    }, ensure_ascii=False)


def start_mission(steps: list[dict[str, str]], personality: str | None = None) -> str:
    """Start a mission. Call it only after you said the mission back to the group in their words and they said yes.
    steps: in order, each {"do": "reach_card"|"find_card"|"survey", "card": <a card>} or {"do": "set_light",
    "color": <a colour>}; personality: one of mission_options's personalities, or none. Refused while another mission
    runs or is paused: stop it first (stop_mission), then wait until mission_status says it stopped."""
    if RUN["thread"] is not None and RUN["thread"].is_alive():
        return json.dumps({"result": "refused", "why": "a mission is running or paused: stop it first (stop_mission), "
                                                       "then wait until mission_status says it stopped"})
    mission = {"steps": steps}
    if personality:
        mission["personality"] = personality
    try:
        strategist.personality(mission.get("personality"))
        words = strategist.task(mission)
    except strategist.Refused as e:
        return json.dumps({"result": "refused", "why": str(e)})
    mission_id = time.strftime("%Y%m%d-%H%M%S")
    RUN.update(id=mission_id, report=None, error=None, started=time.monotonic(), task=words)
    TURN["paused_in"] = None  # a pause of an earlier mission does not count for this one
    say("go")

    def run():
        try:
            RUN["report"] = strategist.run(mission, SETUP["model"], SETUP["key"], SETUP["tools"],
                                           out=STATE_DIR / "missions" / mission_id, cap_usd=SETUP["cap"])
        except Exception as e:  # noqa: BLE001  (said to the Interpreter, never raised in its face)
            RUN["error"] = f"{type(e).__name__}: {e}"

    RUN["thread"] = threading.Thread(target=run, daemon=True)
    RUN["thread"].start()
    return json.dumps({"result": "started", "mission": mission_id, "what_the_strategist_was_told": words})


def pause_mission() -> str:
    """Pause the mission: the robot stops on the next cell, as soon as possible, and waits. Use it while you clarify a
    new request with the group."""
    if state() not in ("running", "pausing", "paused", "waiting for an adult"):
        return json.dumps({"result": "refused", "why": f"no mission running ({state()})"})
    if state() in ("running", "waiting for an adult"):
        say("pause")
    TURN["paused_in"], TURN["ordered"] = TURN["n"], True
    return json.dumps({"result": state(), "mission": RUN["id"],
                       "what_to_do": "call mission_status: it says paused, with the cell where the robot stopped, once "
                                     "nothing moves; until then, the robot is still finishing its cell"})


def resume_mission() -> str:
    """Resume the paused mission, unchanged."""
    if state() not in ("pausing", "paused"):
        return json.dumps({"result": "refused", "why": f"no mission paused ({state()})"})
    say("go")
    TURN["paused_in"] = None  # the group chose to go on: a later stop needs a new pause, and a new question
    TURN["ordered"] = True
    return json.dumps({"result": "running", "mission": RUN["id"]})


def stop_mission() -> str:
    """Stop the mission: the robot stops on the next cell, as soon as possible. Then mission_status says when it
    stopped."""
    if state() not in ("running", "pausing", "paused", "waiting for an adult"):
        return json.dumps({"result": "refused", "why": f"no mission running ({state()})"})
    if TURN["n"] and not TURN["stop_asked"]:
        if TURN["paused_in"] == TURN["n"]:
            return json.dumps({"result": "refused", "why": ASK_FIRST})
        if TURN["paused_in"] is None:
            return json.dumps({"result": "refused", "why": PAUSE_FIRST})
    say("stop")
    TURN["ordered"] = True
    return json.dumps({"result": "stopping", "mission": RUN["id"]})


def turn(group_asked_to_stop=False):
    """A new message of the group (the filter of Open WebUI): the turns start again, and the mission as it is now."""
    TURN.update(n=TURN["n"] + 1, stop_asked=bool(group_asked_to_stop), ordered=False)
    return json.dumps(status(), ensure_ascii=False)


def mission_status() -> str:
    """Where the mission is: "running", "waiting for an adult" (a move stopped on the way: an adult must put the robot
    back and press Resume, as what_to_do says; the mission goes on after that), "pausing" (asked, the robot is finishing
    its cell), "paused" (nothing moves: with the cell where it stands), "ended", "stopped", "failed" or "no mission
    yet"; when it ended, the report: outcome
    (done, impossible, stopped…), the cards asked about with their cells, why, the moves left. Say the report to the
    group in their words."""
    answer, now = status(), time.monotonic()
    again = TURN["read_in"] == TURN["n"] if TURN["n"] else now - TURN["read_at"] < 10
    if again and not TURN["ordered"] and answer["state"] in ("running", "waiting for an adult"):
        answer["note"] = READ_AGAIN
    TURN.update(read_in=TURN["n"], read_at=now)
    return json.dumps(answer, ensure_ascii=False)


def status():
    """mission_status's answer, as a table (the filter reads it too, without counting as a read of the model)."""
    answer = {"mission": RUN["id"], "state": state(), "what_the_strategist_was_told": RUN["task"]}
    if answer["state"] == "paused":
        answer["robot_stands_on"] = common.held_where()
    if answer["state"] == "waiting for an adult":
        answer["what_to_do"] = (common.waiting_info() or {}).get("what_to_do")
    if RUN["started"] is not None:
        answer["seconds"] = round(time.monotonic() - RUN["started"])
    if RUN["report"] is not None and state() in ("ended", "stopped"):
        answer["report"] = RUN["report"]
    if RUN["error"]:
        answer["why"] = RUN["error"]
    return answer


TOOLS = (mission_options, start_mission, pause_mission, resume_mission, stop_mission, mission_status)


async def answer(send, status, body, kind=b"application/json"):
    await send({"type": "http.response.start", "status": status, "headers": [(b"content-type", kind)]})
    await send({"type": "http.response.body", "body": body})


def bearer_only(app, token, hosts=()):
    """An ASGI app that answers 401 to any HTTP request without "Authorization: Bearer <token>". It also serves
    POST /turn (the filter of Open WebUI), with the same locks as the tools: the token, an allowed Host, no Origin."""
    expected = f"Bearer {token}".encode()

    async def guarded(scope, receive, send):
        if scope["type"] == "http":
            headers = dict(scope.get("headers") or [])
            if not hmac.compare_digest(headers.get(b"authorization", b""), expected):
                await send({"type": "http.response.start", "status": 401,
                            "headers": [(b"content-type", b"text/plain"), (b"www-authenticate", b"Bearer")]})
                await send({"type": "http.response.body", "body": b"Unauthorized\n"})
                return
            if scope.get("path") == "/turn":
                if headers.get(b"host", b"").decode(errors="replace") not in hosts or b"origin" in headers:
                    await answer(send, 403, b"Forbidden\n", b"text/plain")
                elif scope.get("method") != "POST":
                    await answer(send, 405, b"Method Not Allowed\n", b"text/plain")
                else:
                    raw, more = b"", True
                    while more:
                        message = await receive()
                        raw += message.get("body", b"")
                        more = message.get("more_body", False) and len(raw) < 4096
                    try:
                        asked = bool(json.loads(raw or b"{}").get("group_asked_to_stop"))
                    except (ValueError, AttributeError):
                        asked = False
                    await answer(send, 200, turn(asked).encode())
                return
        await app(scope, receive, send)
    return guarded


async def serve(args, token):
    import uvicorn
    from mcp.server.mcpserver import MCPServer
    from mcp.server.transport_security import TransportSecuritySettings
    from mcp.types import ToolAnnotations

    server = MCPServer("robot-mission")
    for tool in TOOLS:
        server.tool(annotations=ToolAnnotations(read_only_hint=tool in (mission_options, mission_status)))(tool)
    logging.disable(logging.INFO)  # the SDK logs each session at INFO; warnings (a refused Host header) stay
    security = TransportSecuritySettings(enable_dns_rebinding_protection=True, allowed_hosts=args.allowed_host,
                                         allowed_origins=[])
    app = server.streamable_http_app(transport_security=security, host=args.host)
    print(f"READY robot-mission: http://{args.host}:{args.port}/mcp, Strategist {SETUP['model']} with the "
          f"{SETUP['tools']} tools, hosts {', '.join(args.allowed_host)}", flush=True)
    config = uvicorn.Config(bearer_only(app, token, set(args.allowed_host)), host=args.host, port=args.port,
                            log_level="warning", access_log=False)
    await uvicorn.Server(config).serve()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--allowed-host", action="append", required=True)
    parser.add_argument("--model", default="none", help="the Strategist's model: none, fake, ollama:<model>, "
                                                        "albert:<model>, or a model of OpenRouter (a key on stdin)")
    parser.add_argument("--tools", default="route", choices=["route", "goal", "step"])
    parser.add_argument("--cap", type=float, default=strategist.RUN_CAP_USD, help="the most a mission may cost, in US "
                                                                                  "dollars")
    args = parser.parse_args()
    token = os.environ.pop("MCP_TOKEN", "")
    if len(token) < 32:
        sys.exit("interpreter_link: MCP_TOKEN missing or too short (32 characters at least)")
    key = "" if args.model in ("none", "fake") or args.model.startswith("ollama:") else sys.stdin.read().strip()
    SETUP.update(model=args.model, key=key, tools=args.tools, cap=args.cap)
    common.CALLER = "strategist"
    import anyio
    anyio.run(serve, args, token)


if __name__ == "__main__":
    main()
