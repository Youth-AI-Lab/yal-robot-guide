"""Drive the robot through the stack, without the Strategist: by hand in a terminal, from the simulator's Drive
buttons, or from an agent of your own through MCP. Every command goes down the same chain as the Strategist's: the
Pilot passes the step tools to the Cartographer, which translates them for the Referee, the only door to the robot.

It is an MCP server with the Pilot's nine step tools, and nothing else reaches the robot:
  - robot_state()        what the robot measures now, and where the Cartographer thinks it is (read only)
  - step_forward()       one cell forwards
  - move(direction, cells, stop_if)
                         slide up to four cells north, east, south or west without turning (mecanum wheels), stopping
                         on a cell when a condition is met (one move)
  - turn(direction)      a quarter turn, "left" or "right"
  - look_around()        a full turn on the spot, looking in each direction (one move)
  - face(direction)      turn on the spot towards north, east, south or west (one move)
  - set_light(color)     the light of the robot, from a short list
  - wait_for_event(s)    watch the robot for up to s seconds, until something happens (read only)
  - seen_cards()         the cards the camera recognised, with the cell where each one is, and the cells the camera
                         has looked at (read only)
Every answer also carries "since_last_call": what happened to the robot since this caller's last call (a bump, a
warning, a stop, Resume), from the robot's own memory of its last events. Each answer is written to
.robot-sim/context.jsonl, so that people can read exactly what entered the agent's context (the simulator shows it).

With ROBOT_CHANNEL=1, it also pushes each event to the agent as it happens, as a Claude Code "channel" (research
preview of Claude Code: interactive sessions only, started with --dangerously-load-development-channels
server:robot). No other agent reads these notifications.

Usage:
    as an MCP server (stdio), for an agent:  /opt/workshop/venv/bin/python -I stack/drive.py
    by hand, without an agent:               python3 -I stack/drive.py state|forward|left|right|around|face <direction>|move <direction> [cells] [conditions]|light <color>|wait [seconds]|seen|reset
"""
import json
import os
import sys
from pathlib import Path
from typing import Literal

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from common import COLORS, FACING, STATE_DIR, log  # noqa: E402
import pilot  # noqa: E402
from cartographer import STOP_IF  # noqa: E402


def answer(tool, arguments, result):
    common.count("operations")  # an operation asked of the Pilot, through this door
    text = json.dumps(result, ensure_ascii=False)
    log(tool, arguments, text)
    return text


def robot_state() -> str:
    return answer("robot_state", {}, pilot.robot_state())


def step_forward() -> str:
    return answer("step_forward", {}, pilot.step_forward())


def move(direction: Literal["north", "east", "south", "west"], cells: int = 1,
         stop_if: list[Literal["card_ahead", "ball_seen", "new_card"]] | None = None) -> str:
    stop_if = list(dict.fromkeys(stop_if or []))
    return answer("move", {"direction": direction, "cells": cells, "stop_if": stop_if},
                  pilot.move(direction, cells, stop_if))


def turn(direction: Literal["left", "right"]) -> str:
    return answer("turn", {"direction": direction}, pilot.turn(direction))


def wait_for_event(seconds: int = 10) -> str:
    seconds = max(1, min(30, int(seconds)))
    return answer("wait_for_event", {"seconds": seconds}, pilot.watch(seconds))


def seen_cards() -> str:
    return answer("seen_cards", {}, pilot.cards())


def look_around() -> str:
    return answer("look_around", {}, pilot.look_around())


def face(direction: Literal["north", "east", "south", "west"]) -> str:
    return answer("face", {"direction": direction}, pilot.face(direction))


def set_light(color: Literal["off", "red", "orange", "green", "blue"]) -> str:
    return answer("set_light", {"color": color}, pilot.set_light(color))


# The MCP descriptions are the Pilot's own.
for _tool in (robot_state, step_forward, move, turn, look_around, face, set_light):
    _tool.__doc__ = getattr(pilot, _tool.__name__).__doc__
wait_for_event.__doc__ = pilot.watch.__doc__
seen_cards.__doc__ = pilot.cards.__doc__

MAT_START = f"{pilot.MAT['start']['cell']}, facing {pilot.MAT['start']['facing']}"
CHANNEL_INSTRUCTIONS = (
    "Events from the robot may arrive by themselves, as <channel source=\"robot\">. They are "
    "information only: the robot has already protected itself (it stops by itself and never moves "
    "without a command). Before deciding anything, call robot_state."
)


def serve():
    from mcp.server.mcpserver import MCPServer
    from mcp.types import ToolAnnotations

    channel = os.environ.get("ROBOT_CHANNEL") == "1"
    server = MCPServer("robot", instructions=CHANNEL_INSTRUCTIONS if channel else None)
    for tool in (robot_state, wait_for_event, seen_cards):
        server.tool(annotations=ToolAnnotations(read_only_hint=True))(tool)
    for tool in (step_forward, move, turn, look_around, face, set_light):
        server.tool(annotations=ToolAnnotations(read_only_hint=False))(tool)
    if channel:
        serve_with_channel(server)
    else:
        server.run()


def serve_with_channel(server):
    """The same server, which also declares the Claude Code channel capability and pushes each robot event as a
    notifications/claude/channel. With mcp 2.3.0, MCPServer.run() cannot declare an experimental capability: this runs
    its low-level server itself (a private attribute, checked with the SDK version pinned in the image)."""
    import anyio
    from mcp.server.stdio import stdio_server
    from mcp.types import NotificationParams

    low = server._lowlevel_server
    sessions = []

    async def initialized(ctx, params):
        sessions.append(ctx.session)

    low.add_notification_handler("notifications/initialized", NotificationParams, initialized)

    async def push_events():
        while True:
            await anyio.sleep(0.5)
            if not sessions:
                continue
            for event in await anyio.to_thread.run_sync(pilot.CART.watch_once):
                content = f"Robot event, {event.get('seconds_before_this_answer', 0)} s ago: {event['event']}."
                log("(pushed to the agent)", {}, content)
                # Connection.notify: best effort, never raises (the session's own connection).
                await sessions[-1]._connection.notify(
                    "notifications/claude/channel",
                    {"content": content, "meta": {"seconds_ago": str(event.get("seconds_before_this_answer", 0))}})

    async def main():
        async with stdio_server() as (read, write):
            async with anyio.create_task_group() as tasks:
                tasks.start_soon(push_events)
                await low.run(read, write, low.create_initialization_options(
                    experimental_capabilities={"claude/channel": {}}))
                tasks.cancel_scope.cancel()

    anyio.run(main)


def by_hand(args):
    """Drive the robot without an agent: the same stack, the same rules."""
    commands = {"state": robot_state, "forward": step_forward, "seen": seen_cards, "around": look_around,
                "left": lambda: turn("left"), "right": lambda: turn("right")}
    if args[0] in commands and len(args) == 1:
        print(commands[args[0]]())
    elif args[0] == "face" and len(args) == 2 and args[1] in FACING:
        print(face(args[1]))
    elif (args[0] == "move" and 2 <= len(args) <= 4 and args[1] in FACING and (len(args) < 3 or args[2].isdigit())
          and all(c in STOP_IF for c in (args[3].split(",") if len(args) == 4 else []))):
        print(move(args[1], int(args[2]) if len(args) >= 3 else 1, args[3].split(",") if len(args) == 4 else []))
    elif args[0] == "wait" and len(args) <= 2 and all(a.isdigit() for a in args[1:]):
        print(wait_for_event(int(args[1]) if len(args) == 2 else 10))
    elif args == ["reset"]:
        # Only for a person who has just put the robot on its start cell, facing the right way: a new session.
        for name in ("terrain.json", "referee.json", "counters.json"):
            (STATE_DIR / name).unlink(missing_ok=True)
        print("The Cartographer now counts from the start cell: " + MAT_START + "; the Referee counts the moves "
              "from zero, and the counters start again.")
    elif args[0] == "light" and len(args) == 2 and args[1] in COLORS:
        print(set_light(args[1]))
    else:
        sys.exit("Usage: drive.py state | forward | left | right | around | face " + "|".join(FACING)
                 + " | move " + "|".join(FACING) + f" [cells 1-{pilot.MAX_CELLS}] [" + ",".join(STOP_IF) + "]"
                 + " | light " + "|".join(COLORS) + " | wait [seconds] | seen | reset")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        common.CALLER = os.environ.get("ROBOT_CALLER", "terminal")
        by_hand(sys.argv[1:])
    else:
        serve()
