"""The Strategist: chooses the next tool, following the group's personality, and keeps the mission's progress.
Deliberative layer, about a minute per cycle (choose a goal, have it done, see the result).

It is a loop (this file: code) and the model it calls. The loop opens a new conversation for each mission (system
message: the rules and the personality's instructions; first message: the mission, in words built here from a closed
format), offers the model the Pilot's tools of one kind, runs each call through the Pilot, and stops when the model
calls `report`, or at its caps (turns, time, money). The model never receives the group's words: only the closed
mission. Its answer never leaves as free text: `report` has closed fields, which the Interpreter puts in the group's
words.

A mission (from the Interpreter; today from stack/mission.py), JSON:
    {"steps": [{"do": "reach_card", "card": "barbecue"},   stop right in front of this card, facing it
               {"do": "set_light", "color": "green"},      set the robot's light
               {"do": "find_card", "card": "picnic"},      find out whether this card is on the mat, and where
               {"do": "survey", "card": "dry-grass"}],     find every card of this kind, with its cell
     "personality": "cautious"}                           a file of stack/personality/, or a list of bricks
Its report (to the Interpreter): {"outcome": "done" | "impossible" | "no report", "cards": [{"card", "cell"}],
"why": a reason from a closed list, "moves_left", "turns", "spent_usd"}.
Pause: the Interpreter (stack/interpreter_link.py, or stack/mission.py pause|resume|stop by hand) writes
.robot-sim/control.json; the loop reads it before each call of the model and each tool, and the Pilot before each move,
without asking the model. A move under way ends first: it stops on a cell.

Models: "none" (no model: each step goes straight to the Pilot's goal tools), "fake" (a scripted model: checks the
plumbing, no network), "ollama:<model>" (Ollama on this computer, no key), "albert:<model>" (Albert API of the DINUM,
for French public servants: no price, content not kept, hosted in France; for example albert:deepseek-v4-flash,
albert:gemma-4-31b-it, albert:mistral-small-3-2-24b-instruct-2506; GET /v1/models lists them, and they change), or a
model of OpenRouter, "openrouter:<id>" or a short name of MODELS (data_collection: deny, which does not stop the
provider from keeping what it receives). Any key on stdin, never on the command line, never in a file.
"""
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from common import COLORS, STATE_DIR, log  # noqa: E402
import pilot  # noqa: E402

HERE = Path(__file__).resolve().parent
ALBERT_URL = "https://albert.api.etalab.gouv.fr/v1/chat/completions"
MODELS = {"qwen-flash": "qwen/qwen3.8-flash", "deepseek": "deepseek/deepseek-v4-flash",
          "gemini-argon": "google/gemini-4-argon", "sonnet": "anthropic/claude-sonnet-5.5"}
MAX_TURNS, TIMEOUT_S, RUN_CAP_USD, MAX_TOKENS = 60, 1800, 0.25, 16384  # 4096 cut Qwen's reasoning (bench, 11:30)
FAILED_CALL_USD = 0.01  # a call that failed may have been billed: counted at least this
STEPS = {"reach_card": "card", "set_light": "color", "find_card": "card", "survey": "card"}
WHY = ["", "not on the mat", "not seen yet", "no moves left", "no way there", "the robot waits for an adult", "other"]
RULES = """# Rules for the Strategist, the part of the robot that chooses what to do next

The robot keeps its own safety rules: the Reflex and the Referee. These only guide you.

1. `go_to_cell` takes the robot to a cell by the shortest way it knows, without looking around on the way. It never
   drives onto a card it knows, and stops by itself in front of a card or of an obstacle.
2. `look_around` turns all the way round: the camera sees far, in a cone, and places the cards it recognises.
   `face` turns towards north, east, south or west.
3. To stop right in front of a card, go to a cell next to it, then `face` it.
4. When the robot waits for an adult, wait for the adult with `wait_for_event`, then go on with the mission.
5. When the mission is done, or cannot be done, call `report`. Never end without it.
"""


class Refused(Exception):
    pass


# --- the mission ----------------------------------------------------------------------------------------------
def personality(spec):
    """The bricks of a personality, checked against the library: (Pilot settings, instructions to the model)."""
    library = common.load_json(HERE / "personality" / "bricks.json")["bricks"]
    if isinstance(spec, str):
        path = HERE / "personality" / f"{spec}.json"
        if not spec.replace("-", "").replace("_", "").isalnum() or not path.exists():
            raise Refused(f"no personality {spec!r} in stack/personality/")
        spec = common.load_json(path)["bricks"]
    settings, texts = {}, []
    for name in spec or []:
        brick = library.get(name)
        if brick is None:
            raise Refused(f"no brick {name!r} in the library")
        if brick["kind"] == "pilot":
            settings.update(brick["settings"])
        else:
            texts.append(brick["text"])
    return settings, texts


def task(mission):
    """The mission in words, built only from its closed fields: the model never receives free text."""
    cards = pilot.MAT["cards"]
    if not isinstance(mission, dict) or not isinstance(mission.get("steps"), list) or not mission["steps"]:
        raise Refused("a mission is a list of steps")
    words = []
    for step in mission["steps"]:
        do = step.get("do") if isinstance(step, dict) else None
        if do not in STEPS or set(step) != {"do", STEPS[do]}:
            raise Refused(f"a step is one of {', '.join(STEPS)}, with one argument: {step!r}")
        value = step[STEPS[do]]
        if STEPS[do] == "card" and value not in cards:
            raise Refused(f"no card {value!r} on the list of cards the camera knows (mat.json)")
        if STEPS[do] == "color" and value not in COLORS:
            raise Refused(f"no colour {value!r}")
        words.append({
            "reach_card": f'Go to the card "{value}": stop right in front of it, facing it.',
            "set_light": f"Set the light {value}.",
            "find_card": f'Find out whether the card "{value}" is on the mat, and on which cell.',
            "survey": f'Find every card "{value}" on the mat (there may be several), with the cell of each one.',
        }[do])
    return "You drive a small robot on a mat with the robot tools. " + " Then ".join(words)


# --- the tools ------------------------------------------------------------------------------------------------
def schema(name, function, properties, required):
    return {"type": "function", "function": {"name": name, "description": " ".join((function.__doc__ or "").split()),
            "parameters": {"type": "object", "properties": properties, "required": list(required),
                           "additionalProperties": False}}}


REPORT = {"type": "function", "function": {
    "name": "report",
    "description": "End the mission and say how it went: outcome done (every step done) or impossible; the cards the "
                   "mission asked about that the camera saw, each with its cell; why, when it is impossible. Call it "
                   "once, at the end.",
    "parameters": {"type": "object", "additionalProperties": False, "required": ["outcome"], "properties": {
        "outcome": {"type": "string", "enum": ["done", "impossible"]},
        "cards": {"type": "array", "items": {"type": "object", "additionalProperties": False,
                                             "required": ["card", "cell"], "properties": {
                                                 "card": {"type": "string", "enum": sorted(pilot.MAT["cards"])},
                                                 "cell": {"type": "string", "enum": pilot.CELLS}}}},
        "why": {"type": "string", "enum": WHY}}}}}


def invalid(spec, arguments):
    """Why these arguments do not fit the tool's schema, or None (what an MCP server answers before the call)."""
    if not isinstance(arguments, dict):
        return "the arguments are not a JSON object"
    for key in spec["required"]:
        if key not in arguments:
            return f"missing argument {key}"
    for key, value in arguments.items():
        prop = spec["properties"].get(key)
        if prop is None:
            return f"unknown argument {key}"
        if prop["type"] == "string" and (not isinstance(value, str) or value not in prop["enum"]):
            return f"{key} must be one of {', '.join(prop['enum'])}"
        if prop["type"] == "integer" and (not isinstance(value, int) or isinstance(value, bool)
                                          or not prop["minimum"] <= value <= prop["maximum"]):
            return f"{key} must be a whole number from {prop['minimum']} to {prop['maximum']}"
        if prop["type"] == "array":
            items = prop["items"]
            if not isinstance(value, list):
                return f"{key} must be a list"
            for v in value:
                if items["type"] == "string" and v not in items["enum"]:
                    return f"{key} must be a list of {', '.join(items['enum'])}"
                if items["type"] == "object" and invalid(items, v):
                    return f"{key}: {invalid(items, v)}"
    return None


# --- the models -----------------------------------------------------------------------------------------------
class ApiError(Exception):
    def __init__(self, exit_code, detail):
        super().__init__(detail)
        self.exit_code = exit_code


def fake_model(mission, messages):
    """A scripted model: look, read the record, report what it says. No network, no cost."""
    plan = [("robot_state", {}), ("look_around", {}), ("seen_cards", {})]
    done = sum(1 for m in messages if m["role"] == "tool")
    if done < len(plan):
        name, arguments = plan[done]
        return {"role": "assistant", "content": None, "tool_calls": [
            {"id": f"call_{done}", "type": "function", "function": {"name": name, "arguments": json.dumps(arguments)}}]
        }, {}, "tool_calls"
    record = json.loads(messages[-1]["content"]).get("cards_seen", [])
    asked = {s.get("card") for s in mission["steps"]}
    found = [{"card": c["card"], "cell": c["cell"]} for c in record if c["card"] in asked]
    arguments = {"outcome": "done" if found else "impossible", "cards": found, "why": "" if found else "not on the mat"}
    return {"role": "assistant", "content": None, "tool_calls": [
        {"id": f"call_{done}", "type": "function", "function": {"name": "report", "arguments": json.dumps(arguments)}}]
    }, {}, "tool_calls"


def api_model(model, messages, tools, key, usage, flush):
    if model.startswith("ollama:"):
        url, body = "http://127.0.0.1:11434/v1/chat/completions", {"model": model[7:]}
        headers = {"Content-Type": "application/json"}
    elif model.startswith("albert:"):
        # Albert API (DINUM), OpenAI-compatible: for French public servants (ProConnect). No price; the content is not
        # kept (metadata are, 24 months); hosted in France. tool_choice "auto" is required by deepseek-v4-flash.
        url, body = ALBERT_URL, {"model": model[7:]}
        headers = {"Content-Type": "application/json", "Authorization": "Bearer " + key}
    else:
        name = model[11:] if model.startswith("openrouter:") else MODELS[model]
        url = "https://openrouter.ai/api/v1/chat/completions"
        body = {"model": name, "provider": {"data_collection": "deny"}, "usage": {"include": True}}
        headers = {"Content-Type": "application/json", "Authorization": "Bearer " + key}
    body.update(messages=messages, tools=tools, tool_choice="auto", parallel_tool_calls=False, max_tokens=MAX_TOKENS)
    data = json.dumps(body).encode()
    detail = ""
    for attempt in range(3):
        try:
            reply = json.loads(urllib.request.urlopen(urllib.request.Request(url, data=data, headers=headers),
                                                      timeout=300).read())
            choice = reply["choices"][0]
            return choice["message"], reply.get("usage", {}), choice.get("finish_reason")
        except urllib.error.HTTPError as e:
            detail = f"HTTP {e.code}: {e.read()[:300].decode(errors='replace')}"
            if e.code in (401, 402, 403):
                raise ApiError(f"api refused {e.code}", detail)
            if e.code not in (408, 429) and e.code < 500:
                raise ApiError("api error", detail)
            wait = int(e.headers.get("Retry-After") or 0) if str(e.headers.get("Retry-After", "")).isdigit() else 0
            time.sleep(max(wait, 5 * (attempt + 1)))
        except (TimeoutError, OSError) as e:  # no answer: counted as billed, then sent again
            usage["failed_calls"] += 1
            flush()
            detail = f"{type(e).__name__}: no answer from the model's API"
            time.sleep(5 * (attempt + 1))
        except (KeyError, IndexError, ValueError) as e:
            usage["failed_calls"] += 1
            flush()
            detail = f"{type(e).__name__}: the API's answer could not be read"
            time.sleep(5 * (attempt + 1))
    raise ApiError("api error", detail)


# --- the loop -------------------------------------------------------------------------------------------------
def paused():
    """The pause, handled by the loop without the model: wait until resume or stop. True when stopped."""
    while common.mission_control() == "pause":
        time.sleep(0.5)
    return common.mission_control() == "stop"


def without_model(mission):
    """Model "none": each step goes straight to the Pilot's goal tools, no model (the Pilot alone of the bench of
    6 November). The same report, built by code."""
    start, cards, outcome, why = time.monotonic(), [], "done", ""
    for step in mission["steps"]:
        do = step["do"]
        if do == "set_light":
            result = pilot.light(step["color"])
        elif do == "reach_card":
            result = pilot.go_to_card(step["card"])
        else:
            result = pilot.explore(step["card"] if do == "find_card" else None)
        if do in ("find_card", "survey"):
            cards += [{"card": step["card"], "cell": c} for c in result.get("cards_seen", {}).get(step["card"], [])]
            if not cards:
                # "Not on the mat" only when the camera has looked at every cell it can: else, not seen yet.
                left = pilot.cards().get("camera_coverage", {}).get("not_looked_at_yet", [None])
                outcome, why = "impossible", "not seen yet" if left else "not on the mat"
        elif do == "reach_card" and result.get("result") == "done":
            col, row = common.cell_xy(result["cell"])
            dc, dr = pilot.STEP[result["facing"]]
            cards.append({"card": step["card"], "cell": common.cell_name(col + dc, row + dr)})  # right in front
        if common.mission_control() == "stop":
            outcome, why = "stopped", ""
            break
        if result.get("result") not in ("done", "found", "not found"):
            said = result.get("why", "")
            outcome = "impossible"
            why = ("no moves left" if "moves" in said else "the robot waits for an adult" if "adult" in said
                   else "no way there" if "find itself" in said or "way" in said else "other")
            break
    return {"outcome": outcome, "cards": cards, "why": why, "turns": 0,
            "seconds": round(time.monotonic() - start, 1), "spent_usd": 0.0}


def run(mission, model="fake", key="", tools="route", out=None, cap_usd=RUN_CAP_USD):
    """One mission, in a new conversation. Returns the report."""
    settings, texts = personality(mission.get("personality"))
    words = task(mission)
    pilot.PILOT.configure(settings)
    pilot.OBEY_MISSION_CONTROL = True
    out = Path(out or STATE_DIR / "missions" / time.strftime("%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    if model == "none":
        common.CALLER = "strategist"
        (out / "mission.json").write_text(json.dumps(dict(mission, task=words, settings=pilot.PILOT.settings), indent=1))
        report = without_model(mission)
        report["moves_left"] = pilot.robot_state().get("moves_left")
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n")
        return report
    toolset = pilot.TOOLSETS[tools]
    offered = [schema(name, fn, props, req) for name, (fn, _, props, req) in toolset.items()] + [REPORT]
    specs = {t["function"]["name"]: t["function"]["parameters"] for t in offered}
    system = RULES + ("\n## How you drive\n\n" + "\n".join(f"- {t}" for t in texts) + "\n" if texts else "")
    messages = [{"role": "system", "content": system}, {"role": "user", "content": words}]
    usage = {"prompt_tokens": 0, "completion_tokens": 0, "cost": 0.0, "api_calls": 0, "failed_calls": 0,
             "dearest_call": 0.0}

    def spent():
        return usage["cost"] + usage["failed_calls"] * max(usage["dearest_call"], FAILED_CALL_USD)

    def flush():
        (out / "usage.json").write_text(json.dumps(dict(usage, spent=spent())))

    common.CALLER = "strategist"
    (out / "mission.json").write_text(json.dumps(dict(mission, task=words, settings=pilot.PILOT.settings), indent=1))
    transcript = (out / "transcript.jsonl").open("w")
    for m in messages:
        transcript.write(json.dumps(m, ensure_ascii=False) + "\n")
    report, outcome, turns, nudged, start = None, None, 0, False, time.monotonic()
    flush()
    try:
        while report is None:
            if paused():
                outcome = "stopped"
                break
            if turns >= MAX_TURNS or time.monotonic() - start > TIMEOUT_S:
                outcome = "limit"
                break
            if spent() > cap_usd:
                outcome = "cost cap"
                break
            turns += 1
            if model == "fake":
                message, used, finish = fake_model(mission, messages)
            else:
                message, used, finish = api_model(model, messages, offered, key, usage, flush)
            usage["api_calls"] += 1
            for k in ("prompt_tokens", "completion_tokens"):
                usage[k] += used.get(k) or 0
            usage["cost"] += float(used.get("cost") or 0)
            usage["dearest_call"] = max(usage["dearest_call"], float(used.get("cost") or 0))
            flush()
            message = {k: v for k, v in message.items() if k in ("content", "tool_calls", "reasoning", "reasoning_details")}
            message["role"] = "assistant"
            messages.append(message)
            transcript.write(json.dumps(dict(message, finish_reason=finish), ensure_ascii=False) + "\n")
            if not message.get("tool_calls"):
                if nudged:
                    outcome = "no report"
                    break
                nudged = True  # once: the model answered in words; the report is a tool
                messages.append({"role": "user", "content": "Call report to end the mission."})
                continue
            moved = False
            for n, call in enumerate(message["tool_calls"]):
                call.setdefault("id", f"call_{turns}_{n}")
                name = (call.get("function") or {}).get("name", "")
                try:
                    arguments = json.loads((call.get("function") or {}).get("arguments") or "{}")
                except ValueError:
                    arguments = None
                why = "no tool named " + name if name not in specs else invalid(specs[name], arguments)
                if why:
                    result = {"result": "refused", "why": f"invalid arguments: {why}: nothing was sent"}
                elif name == "report":
                    report = arguments
                    result = {"result": "done"}
                elif paused():
                    outcome, result = "stopped", {"result": "refused", "why": "the mission was stopped"}
                elif name in pilot.MOVES and moved:
                    result = {"result": "refused", "why": "another move is still running: send one move at a time, "
                                                          "and read its answer before the next"}
                else:
                    try:
                        result = toolset[name][0](**arguments)
                    except Exception as e:  # noqa: BLE001  (as an MCP server: the error goes back to the model)
                        result = {"result": "error", "why": f"{type(e).__name__}: {e}"}
                    moved = moved or name in pilot.MOVES
                text = json.dumps(result, ensure_ascii=False)
                log(name, arguments, text, caller="strategist")
                messages.append({"role": "tool", "tool_call_id": call["id"], "content": text})
                transcript.write(json.dumps({"role": "tool", "name": name, "content": text}, ensure_ascii=False) + "\n")
            if outcome == "stopped":
                break
    except ApiError as e:
        outcome = e.exit_code
    transcript.close()
    flush()
    if key:
        for f in out.iterdir():
            if f.is_file() and key in f.read_text(errors="replace"):
                f.write_text("(removed: the output contained the key)")
    state = pilot.robot_state()
    final = dict(report or {"outcome": outcome or "no report", "cards": [], "why": ""},
                 moves_left=state.get("moves_left"), turns=turns, seconds=round(time.monotonic() - start, 1),
                 spent_usd=round(spent(), 4))
    if report is None and outcome:
        final["outcome"] = outcome
    (out / "report.json").write_text(json.dumps(final, ensure_ascii=False, indent=1) + "\n")
    return final


def main():
    """usage: python3 -I stack/strategist.py <mission.json> [model] [route|goal|step]   (the key on stdin)"""
    if len(sys.argv) < 2:
        sys.exit(main.__doc__)
    mission = common.load_json(sys.argv[1])
    model = sys.argv[2] if len(sys.argv) > 2 else "fake"
    key = "" if model == "fake" or model.startswith("ollama:") else sys.stdin.read().strip()
    try:
        print(json.dumps(run(mission, model, key, sys.argv[3] if len(sys.argv) > 3 else "route"), ensure_ascii=False))
    except Refused as e:
        sys.exit(f"refused: {e}")


if __name__ == "__main__":
    main()
