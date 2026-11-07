"""Stand-in for the Interpreter, until it is written: builds a mission in the closed format, sends it to the
Strategist, prints its report; and pauses, resumes or stops the mission running. The Interpreter will do the same
from the group's words, after the group has agreed to the mission (TO BE COMPLETED: MCP over HTTP with a token,
from Open WebUI).

Usage (the folder of the guide is the current folder):
    python3 -I stack/mission.py reach_card:barbecue [set_light:green ...] [--personality cautious] [--model fake]
                                [--tools route|goal|step] [--cap 0.25]
    python3 -I stack/mission.py pause | resume | stop
Steps: reach_card:<card>, set_light:<colour>, find_card:<card>, survey:<card>. Models: none, fake, ollama:<model>,
albert:<model> (Albert API of the DINUM, for French public servants) or a model of OpenRouter; a key is read on stdin
(never on the command line, never in a file of the guide).
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import STATE_DIR  # noqa: E402


def say(do):
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    (STATE_DIR / "control.json").write_text(json.dumps({"do": do}) + "\n", encoding="utf-8")


def main():
    if len(sys.argv) == 2 and sys.argv[1] in ("pause", "resume", "stop"):
        say({"resume": "go"}.get(sys.argv[1], sys.argv[1]))
        print(f"Mission: {sys.argv[1]}.")
        return
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("steps", nargs="+", help="reach_card:<card>, set_light:<colour>, find_card:<card>, survey:<card>")
    parser.add_argument("--personality", default=None, help="a file of stack/personality/ (without .json)")
    parser.add_argument("--model", default="fake")
    parser.add_argument("--tools", default="route", choices=["route", "goal", "step"])
    parser.add_argument("--cap", type=float, default=0.25, help="the most this mission may cost, in US dollars")
    args = parser.parse_args()
    import strategist  # after the arguments: it connects nothing, but reads the mat and the stack's state
    steps = []
    for item in args.steps:
        do, _, value = item.partition(":")
        if do not in strategist.STEPS or not value:
            sys.exit(f"refused: a step is one of {', '.join(strategist.STEPS)}, then a colon and its argument: {item!r}")
        steps.append({"do": do, strategist.STEPS[do]: value})
    mission = {"steps": steps}
    if args.personality:
        mission["personality"] = args.personality
    key = "" if args.model == "fake" or args.model.startswith("ollama:") else sys.stdin.read().strip()
    say("go")
    try:
        report = strategist.run(mission, args.model, key, args.tools, cap_usd=args.cap)
    except strategist.Refused as e:
        sys.exit(f"refused: {e}")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
