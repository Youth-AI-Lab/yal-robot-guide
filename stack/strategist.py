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
"why": a reason from a closed list, "moves_left", "turns", "spent_usd", "counts"}: "counts" are the counters of the
mission (stack/common.py: missions, model_calls, model_usd, operations, commands, cells, moves). The main limits are
on the calls to the model, that is on the cost: MAX_TURNS and the cap in dollars per mission. The adults may add limits
on the robot's counters in stack/rules.json ("limits"); none by default.
Pause: the Interpreter (stack/interpreter_link.py, or stack/mission.py pause|resume|stop by hand) writes
.robot-sim/control.json; the loop reads it before each call of the model and each tool, and the Pilot before each move,
without asking the model; during a move, the Pilot has the robot stop on the next cell, as soon as possible (HOLD,
through the Cartographer and the Referee), with no adult needed.

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
MAX_TURNS, TIMEOUT_S, RUN_CAP_USD, MAX_TOKENS = 100, 3000, 0.25, 16384  # 4096 cut Qwen's reasoning (bench, 11:30); 100 turns: at 60, three absent-card runs of Hermes ended one cell short (2026-11-07)
# The robot waits for an adult (a move stopped on the way): the loop waits, never calling its model, until Resume, a
# stop asked by the Interpreter, or this long; then it calls the model again with the event. The wait does not count in
# TIMEOUT_S.
WAIT_FOR_ADULT_S = 900
FAILED_CALL_USD = 0.01  # a call that failed may have been billed: counted at least this
# The same call refused again and again: the model is told from the second time, the mission ends ("limit") at the
# fourth. Mistral Small 3.2 (Albert, 2026-11-07) sent look_around {"seconds": 1} 44 times, and used up the quota.
SAME_REFUSAL_MAX = 4
BUSY_WAIT_S = 0.5  # the stack held a moment by another program (drive.py, the page): the record is read again
STEPS = {"reach_card": "card", "set_light": "color", "find_card": "card", "survey": "card"}
WHY = ["", "not on the mat", "not seen yet", "no moves left", "a limit of the adults", "no way there",
       "the robot waits for an adult", "the robot does not know where it is", "other"]
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
        if not spec.replace("-", "").replace("_", "").isalnum() or spec == "bricks" or not path.exists():
            raise Refused(f"no personality {spec!r} in stack/personality/")
        spec = common.load_json(path)["bricks"]
    elif spec is not None and not isinstance(spec, list):
        raise Refused(f"a personality is a name of stack/personality/ or a list of bricks: {spec!r}")
    settings, texts = {}, []
    for name in spec or []:
        brick = library.get(name) if isinstance(name, str) else None
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
        if not isinstance(do, str) or do not in STEPS or set(step) != {"do", STEPS[do]}:
            raise Refused(f"a step is one of {', '.join(STEPS)}, with one argument: {step!r}")
        value = step[STEPS[do]]
        if STEPS[do] == "card" and (not isinstance(value, str) or value not in cards):
            raise Refused(f"no card {value!r} on the list of cards the camera knows (mat.json)")
        if STEPS[do] == "color" and (not isinstance(value, str) or value not in COLORS):
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


TOO_EARLY = "the camera has not looked at every cell of the mat yet"
CARD_STEPS = {"reach_card", "find_card", "survey"}


def read_record():
    """The Cartographer's record (seen_cards), three tries if the stack is busy; None if the robot cannot be reached."""
    for attempt in range(3):
        try:
            answer = pilot.command("seen_cards")
        except Exception:  # noqa: BLE001
            return None
        if answer.get("rule") not in ("another_command", "one_move_at_a_time"):
            break
        time.sleep(BUSY_WAIT_S)
    return answer


def early_report(mission, arguments):
    """A report that gives up looking for a card: what the record must back before it is taken."""
    return arguments.get("outcome") == "impossible" and arguments.get("why") in ("not seen yet", "not on the mat") \
        and any(step.get("do") in CARD_STEPS for step in mission.get("steps", []))


def too_early(record):
    """Why giving up on a card comes too early, or None: the record still lists cells the camera has not looked at.
    Qwen 3 Coder (Albert, 2026-11-07) gave up twice with two cells left, once writing that the coverage had none."""
    if not isinstance(record, dict) or "cards_seen" not in record:
        return None
    left = (record.get("camera_coverage") or {}).get("not_looked_at_yet")
    if not left:
        return None  # every cell looked at, or the place on the mat not known (no list)
    shown = ", ".join(left[:8]) + (", …" if len(left) > 8 else "")
    return (f"{TOO_EARLY}: {len(left)} cell{'s' if len(left) > 1 else ''} left ({shown}). Go and look at them, then "
            "report; or call report again if they cannot be reached")


def record_check(cards):
    """Why the cards of a report cannot be taken, or None; and the Cartographer's answer to give back with a refusal.
    Each card must be in the record (seen_cards), on its cell or one of its could_also_be. Mistral Small 3.2 (Albert,
    2026-11-07) reported the barbecue on D4 without ever seeing it, and the group was told. The record is read by a
    seen_cards like any other: what that call heard (since_last_call, notices) goes back to the model with the refusal.
    A robot that cannot be reached at all: nothing to compare with, the report goes through."""
    if not cards:
        return None, None
    answer = read_record()
    if answer is None:
        return None, None
    if "cards_seen" not in answer:
        return "the record of the cards could not be read now: report again", answer
    placed, unplaced = set(), set()
    for seen in answer["cards_seen"]:
        if seen.get("cell"):
            placed.update((seen["card"], cell) for cell in [seen["cell"], *(seen.get("could_also_be") or [])])
        else:
            unplaced.add(seen["card"])
    wrong = [c for c in cards if (c.get("card"), c.get("cell")) not in placed]
    if not wrong:
        return None, answer
    if any(c.get("card") in unplaced for c in wrong):
        return ("the robot does not know where it is now, so the record gives no cells: find its place first, or "
                "report impossible with why \"the robot does not know where it is\""), answer
    named = ", ".join(f"{c.get('card')} on {c.get('cell')}" for c in wrong)
    return f"the record does not place {named}: report only cards and cells that seen_cards gives", answer


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
# Waits before trying a call of the model again. Albert API answered two calls of Qwen 3 Coder with an error that the
# same request, sent again later, did not give (2026-11-08): every HTTP error is tried again, except a refused key.
API_WAITS_S = (10, 30, 60)


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
    for wait_s in API_WAITS_S:
        try:
            reply = json.loads(urllib.request.urlopen(urllib.request.Request(url, data=data, headers=headers),
                                                      timeout=300).read())
            choice = reply["choices"][0]
            return choice["message"], reply.get("usage", {}), choice.get("finish_reason")
        except urllib.error.HTTPError as e:
            detail = f"HTTP {e.code}: {e.read()[:300].decode(errors='replace')}"
            if e.code in (401, 402, 403):
                raise ApiError(f"api refused {e.code}", detail)
            wait = int(e.headers.get("Retry-After") or 0) if str(e.headers.get("Retry-After", "")).isdigit() else 0
            usage["api_errors"] = usage.get("api_errors", []) + [detail[:300]]
            flush()
            time.sleep(max(wait, wait_s))
        except (TimeoutError, OSError) as e:  # no answer: counted as billed, then sent again
            usage["failed_calls"] += 1
            flush()
            detail = f"{type(e).__name__}: no answer from the model's API"
            time.sleep(wait_s)
        except (KeyError, IndexError, ValueError) as e:
            usage["failed_calls"] += 1
            flush()
            detail = f"{type(e).__name__}: the API's answer could not be read"
            time.sleep(wait_s)
    raise ApiError("api error", detail)


# --- the loop -------------------------------------------------------------------------------------------------
def paused():
    """The pause, handled by the loop without the model, between two tools (nothing moves): acknowledged up the chain
    by the Pilot, then wait until resume or stop. True when stopped."""
    pilot.hold_still()
    return common.mission_control() == "stop"


def waits_for_adult(result):
    """A tool's answer that leaves the robot waiting for an adult (a goal or route tool gave up for it, or a step
    tool's move stopped on the way)."""
    if not isinstance(result, dict):
        return False
    state = result.get("state") if isinstance(result.get("state"), dict) else result
    return ("waits for an adult" in str(result.get("why", "")) or result.get("result") == "stopped"
            or state.get("waiting_for_adult") is True)


def wait_for_adult():
    """Wait, without the model, until an adult presses Resume: "resumed" (with where the robot stands), "stopped" (the
    Interpreter stopped the mission), or "too long". The Interpreter reads .robot-sim/waiting.json meanwhile."""
    since = time.monotonic()
    common.say_waiting({"since": time.strftime("%H:%M:%S"), "what_to_do": pilot.CART.adult_instruction()})
    try:
        while time.monotonic() - since < WAIT_FOR_ADULT_S:
            if common.mission_control() == "stop":
                return "stopped", None, time.monotonic() - since
            state = pilot.command("robot_state")
            if isinstance(state, dict) and state.get("waiting_for_adult") is False:
                return "resumed", state, time.monotonic() - since
            time.sleep(1.0)
        return "too long", None, time.monotonic() - since
    finally:
        common.say_waiting(None)


def resumed_event(state):
    """What the model is told when the robot moves again: built by code, closed words only."""
    if state.get("position_known") and state.get("cell"):
        where = f"it stands on {state['cell']} facing {state['facing']}"
    else:
        where = "where it stands on the mat is not known yet"
    return f"Event: an adult put the robot back and pressed Resume; {where}. Go on with the mission."


def without_model(mission):
    """Model "none": each step goes straight to the Pilot's goal tools, no model (the Pilot alone of the bench of
    6 November). The same report, built by code."""
    start, cards, outcome, why = time.monotonic(), [], "done", ""
    for step in mission["steps"]:
        do = step["do"]
        for _ in range(3):  # an adult's help between two tries: the step again, from where the robot is put back
            if do == "set_light":
                result = pilot.operate(pilot.light, color=step["color"])
            elif do == "reach_card":
                result = pilot.operate(pilot.go_to_card, card=step["card"])
            else:
                result = pilot.operate(pilot.explore, look_for=step["card"] if do == "find_card" else None)
            if not waits_for_adult(result):
                break
            how, _, _ = wait_for_adult()
            if how != "resumed":
                result = {"result": "gave up", "why": "the mission was stopped" if how == "stopped"
                          else "the robot waits for an adult"}
                break
        if do in ("find_card", "survey"):
            seen = result.get("cards_seen", {}).get(step["card"], [])
            # Only a cell of the mat: in a frame relative to the start, the Cartographer has no cell to give.
            found = [{"card": step["card"], "cell": c} for c in seen if c in pilot.CELLS]
            cards += found
            if not found:
                # "Not on the mat" only when the camera has looked at every cell it can: else, not seen yet.
                left = pilot.cards().get("camera_coverage", {}).get("not_looked_at_yet", [None])
                # left is None while the place on the mat is unknown: nothing is counted, nothing can be concluded.
                outcome, why = "impossible", ("the robot does not know where it is" if seen or left is None
                                              else "not seen yet" if left else "not on the mat")
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
            why = ("no moves left" if "no moves left" in said else "a limit of the adults" if "the adults allow" in said
                   else "the robot waits for an adult" if "adult" in said
                   else "the robot does not know where it is" if "where it is" in said
                   else "no way there" if "find itself" in said or "way" in said else "other")
            break
    return {"outcome": outcome, "cards": cards, "why": why, "turns": 0,
            "seconds": round(time.monotonic() - start, 1), "spent_usd": 0.0}


def run(mission, model="fake", key="", tools="route", out=None, cap_usd=RUN_CAP_USD):
    """One mission, in a new conversation. Returns the report."""
    words = task(mission)  # first: it refuses a mission that is not an object
    settings, texts = personality(mission.get("personality"))
    pilot.PILOT.configure(settings)
    pilot.OBEY_MISSION_CONTROL = True
    common.mission_begins()
    common.say_waiting(None)  # a mission that ended in the middle of a wait left it
    out = Path(out or STATE_DIR / "missions" / time.strftime("%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    if model == "none":
        common.CALLER = "strategist"
        (out / "mission.json").write_text(json.dumps(dict(mission, task=words, settings=pilot.PILOT.settings), indent=1))
        report = without_model(mission)
        report["moves_left"] = pilot.robot_state().get("moves_left")
        report["counts"] = common.counts()["this_mission"]
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
    questioned = False  # a report giving up on a card was refused once for cells left
    last_refused, same_refused = None, 0
    waited = 0.0  # time spent waiting for an adult: not counted in TIMEOUT_S
    flush()
    try:
        while report is None:
            if paused():
                outcome = "stopped"
                break
            if turns >= MAX_TURNS or time.monotonic() - start - waited > TIMEOUT_S:
                outcome = "limit"
                break
            if spent() > cap_usd:
                outcome = "cost cap"
                break
            turns += 1
            common.count("model_calls")
            if model == "fake":
                message, used, finish = fake_model(mission, messages)
            else:
                message, used, finish = api_model(model, messages, offered, key, usage, flush)
            usage["api_calls"] += 1
            for k in ("prompt_tokens", "completion_tokens"):
                usage[k] += used.get(k) or 0
            usage["cost"] += float(used.get("cost") or 0)
            common.count("model_usd", float(used.get("cost") or 0))
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
            moved, adult, counted = False, False, set()
            for n, call in enumerate(message["tool_calls"]):
                call.setdefault("id", f"call_{turns}_{n}")
                name = (call.get("function") or {}).get("name", "")
                try:
                    arguments = json.loads((call.get("function") or {}).get("arguments") or "{}")
                except ValueError:
                    arguments = None
                why, record = "no tool named " + name if name not in specs else invalid(specs[name], arguments), None
                if why:
                    why = f"invalid arguments: {why}: nothing was sent"
                elif name == "report" and outcome != "limit":
                    why, record = record_check(arguments.get("cards") or [])
                    if not why and not questioned and early_report(mission, arguments):
                        # Giving up on a card with cells the camera has not looked at: refused once, with the cells;
                        # the next report is taken (the model may know they cannot be reached).
                        record = record if record is not None else read_record()
                        why = too_early(record)
                        questioned = bool(why)
                if outcome == "limit":  # the same call refused too often, earlier in this answer: nothing else runs
                    result = {"result": "refused", "why": "the mission ended: the same call was refused "
                                                          f"{SAME_REFUSAL_MAX} times in a row"}
                elif why:
                    result = {"result": "refused", "why": why}
                    if record is not None:
                        result["seen_cards"] = record
                    this = (name, (call.get("function") or {}).get("arguments") or "")  # word for word
                    if this not in counted:  # counted once per answer: the model reads each refusal before it counts
                        counted.add(this)
                        same_refused = same_refused + 1 if this == last_refused else 1
                    last_refused = this
                    if same_refused >= 2:
                        result["why"] += f"; this same call was refused {same_refused} times in a row: change it"
                    if same_refused >= SAME_REFUSAL_MAX:
                        outcome = "limit"
                elif name == "report":
                    last_refused = None
                    report = arguments
                    result = {"result": "done"}
                elif paused():
                    outcome, result = "stopped", {"result": "refused", "why": "the mission was stopped"}
                elif name in pilot.MOVES and moved:
                    result = {"result": "refused", "why": "another move is still running: send one move at a time, "
                                                          "and read its answer before the next"}
                else:
                    last_refused = None
                    try:
                        result = pilot.operate(toolset[name][0], **arguments)
                    except Exception as e:  # noqa: BLE001  (as an MCP server: the error goes back to the model)
                        result = {"result": "error", "why": f"{type(e).__name__}: {e}"}
                    moved = moved or name in pilot.MOVES
                text = json.dumps(result, ensure_ascii=False)
                log(name, arguments, text, caller="strategist")
                messages.append({"role": "tool", "tool_call_id": call["id"], "content": text})
                transcript.write(json.dumps({"role": "tool", "name": name, "content": text}, ensure_ascii=False) + "\n")
                adult = adult or waits_for_adult(result)
            if outcome in ("stopped", "limit"):
                break
            if adult:
                # The model finished its turn; it is not called while the robot waits, then hears the event.
                how, state, seconds = wait_for_adult()
                waited += seconds
                if how != "resumed":
                    outcome = "stopped" if how == "stopped" else "impossible"
                    report = None if how == "stopped" else {"outcome": "impossible", "cards": [],
                                                            "why": "the robot waits for an adult"}
                    break
                event = resumed_event(state)
                messages.append({"role": "user", "content": event})
                transcript.write(json.dumps({"role": "user", "content": event}, ensure_ascii=False) + "\n")
    except ApiError as e:
        outcome = e.exit_code
        usage["api_error"] = str(e)[:300]  # for whoever runs the robot; never in the report, which is closed
    transcript.close()
    flush()
    if key:
        for f in out.iterdir():
            if f.is_file() and key in f.read_text(errors="replace"):
                f.write_text("(removed: the output contained the key)")
    state = pilot.robot_state()
    final = dict(report or {"outcome": outcome or "no report", "cards": [], "why": ""},
                 moves_left=state.get("moves_left"), turns=turns, seconds=round(time.monotonic() - start, 1),
                 spent_usd=round(spent(), 4), counts=common.counts()["this_mission"])
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
