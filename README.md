# A robot driven by an agent: the simulator

## Youth AI Lab - Erasmus+ project

Partners: L.A.B. (Aix-en-Provence, France), UAB (Barcelona, Spain), Perlatecnica (Naples, Italy),
Digitale Wolven (Antwerp, Belgium), CSC Vent des Îles (La Rochelle, France).

> [!WARNING]
> **A prototype, for facilitators.** Nothing here has run on a real robot, and no real model has
> driven it yet. Never use it with young people: the rules for that are not decided.

The young people of the project will build a robot (a STeaMi board and an OpenMV N6 camera) to give
their chatbot a body. Before the robot exists, this simulator lets you try the whole chain: the
program of the robot, the gateway that is the only door to it, and an agent that drives it.

```
 robot (or this simulator)         gateway (MCP server)              agent
 ┌───────────────────────┐  text   ┌──────────────────────┐  MCP   ┌──────────────┐
 │ reflexes: stop, speed │ ◄─────► │ 4 tools, its limits  │ ◄────► │ Claude Code, │
 │ camera: a card label  │  cable  │ writes what the agent│        │ OpenCode,    │
 └───────────────────────┘         │ receives             │        │ Hermes...    │
                                   └──────────────────────┘        └──────────────┘
```

Three rules hold the design together:

- **The robot protects itself.** It stops in front of an obstacle, when the cable comes loose, and
  when someone presses its emergency button, without asking the computer
  ([robot/reflex.py](robot/reflex.py), [robot/firmware.py](robot/firmware.py)).
- **The gateway is the only door.** Four tools, one cell per move, never off the mat, a number of
  moves per session ([gateway/robot_gateway.py](gateway/robot_gateway.py)).
- **No image ever leaves the robot.** The camera gives the name of a card from a short list.

The simulator replaces only the physical part. The program of the robot runs in it unchanged, and
the gateway talks to it through a pseudo-terminal, as it will talk to the robot through its USB
cable.

---

## Part 1 - Start the simulator

1. Open this repository in a Codespace, or on your computer with the
   [dev containers guide](https://github.com/Youth-AI-Lab/yal-devcontainer-guide).
2. In the terminal:

   ```bash
   python3 -I sim/robot_sim.py
   ```

   You should see:

   ```
   Robot simulator: http://127.0.0.1:8780  (in a Codespace: Ports tab, port 8780)
   Simulated USB cable: /dev/pts/3
   READY port=8780 serial=/dev/pts/3
   ```

3. Open the page: in a Codespace, the **Ports** tab, line **Robot simulator (8780)**, then the globe
   icon. Leave this terminal open: closing it stops the simulator.

On the page: the mat seen from above, the robot (blue circle, its light around it, an arrow where it
looks), the cards on the floor, two boxes and a rolling ball. The dashed square is the cell the
camera looks at; the red line is what the distance sensor measures.

---

## Part 2 - Drive it, through the gateway

The robot moves only through the gateway, the same door an agent uses, with the same limits. Two ways
to drive it by hand:

- **On the page**, the **Drive** panel. **North**, **West**, **South**, **East** and the arrow keys of
  the keyboard go towards a side of the mat: the robot turns as needed, then moves one cell (as the
  gateway believes it faces). **↑ Forward**, **↺ Left**, **↻ Right** are the moves an agent has, one at
  a time. **State**, **Light**, **Seen cards** and **Wait for an event (10 s)** too. Each button runs the gateway, as
  the command below would.
- **In a second terminal**:

  ```bash
  python3 -I gateway/robot_gateway.py state
  python3 -I gateway/robot_gateway.py forward
  python3 -I gateway/robot_gateway.py right
  python3 -I gateway/robot_gateway.py light green
  ```

Each answer is what an agent would receive, word for word. On the page, **On the wire** shows the
lines between the computer and the robot, and **What the agent received** shows these answers, with
who sent the order: an agent, a terminal or the Drive buttons. One order at a time: while one runs,
any other is refused, wherever it comes from.

You can drag with the mouse everything that lies on the mat: the boxes (obstacles, brown squares,
**Add a box** adds one, a double-click removes one), the cards (they land on a cell) and the ball
(with its whole path). Not the robot: the gateway counts its moves to know where it is, so a robot
moved by hand would be lost. Press **Start again** to put it back on its start cell.

Things to try:

- Drag a box onto the cell in front of the robot (the dashed square), then **Forward**: refused,
  "something is in front of the robot".
- **Forward** and, while it moves, drag a box right in front of it: the robot stops by itself. The
  gateway then refuses everything: the position is unknown. Press **An adult puts it back and
  presses Resume**, then **Forward** again.
- **Forward** and, while it moves, **Unplug the cable**: the robot stops after one second (its
  watchdog). The gateway waits, then says the move was stopped.
- **Emergency stop**, then **Forward**: refused.
- Turn towards the edge and **Forward**: refused, the next cell is outside the mat.
- Drag a card onto the cell in front of the robot, then **State**: `card_ahead` names it.
- Put the robot on the path of the ball (column E) and wait. When the ball comes close in front, the
  light turns red (the warning reflex); when it touches the robot, it bounces off the bumper. The
  robot does not move away: standing still is its safe state, and moving is a decision. While it
  moves, the ball stops it (distance sensor in front, bumper on the sides).

> [!NOTE]
> The gateway counts the moves to know where the robot is: it never sees the real cell. Set
> **Wheels slip** to 10 %, drive a few cells, and compare **Real cell** on the page with the `cell`
> the gateway gives.

---

## Part 3 - Change a reflex

[robot/reflex.py](robot/reflex.py) holds what the robot does by itself: `must_stop()` while it moves,
and `warn()` while it stands still (the colour of its light when something comes close; only a
colour, the robot never moves by itself). Change `STOP_DISTANCE_CM`, `MAX_SPEED_CM_S` or
`WARN_DISTANCE_CM`, or the functions, save, and press **Reload reflex.py** on the page. If the file has a mistake, the page says so and the old
reflexes keep running.

The program of the robot keeps its own limits ([robot/firmware.py](robot/firmware.py),
`HARD_MAX_SPEED_CM_S`, `HARD_MIN_DISTANCE_CM`, and the bumper all around it, which stops every move):
`reflex.py` can make the robot more careful, never less careful than those.

Both files are written in the part of Python that MicroPython understands, so that the same files
can run on the robot's board.

---

## Part 4 - Real time or frozen time

The **Time** list changes how the world behaves while the agent thinks:

- **real time**: the ball keeps rolling while nobody moves, as in a room. A model takes seconds to
  decide; meanwhile the world changes.
- **frozen**: the world waits for the robot, as in a video game. Agents that play games, like
  [pokemon-agent](https://github.com/NousResearch/pokemon-agent), live in this world.

This is why the reflexes are on the robot: a study in a simulator measured that decisions of 2.5 to
14 seconds make language-model agents fail in a world that does not wait, and that fast fixed rules
combined with a slower model do better
([Zheng et al., arXiv 2506.07223](https://arxiv.org/abs/2506.07223)).

---

## Part 5 - Connect an agent (not tested with a real model)

The gateway is an MCP server. Its six tools:

| Tool | What it does | Read only |
|---|---|---|
| `robot_state()` | distance in front, card in front, battery, buttons, the cell where the gateway thinks the robot is | yes |
| `step_forward()` | one cell forwards, then stop | no |
| `turn(direction)` | a quarter turn, `left` or `right` | no |
| `set_light(color)` | `off`, `red`, `orange`, `green` or `blue` | no |
| `wait_for_event(seconds)` | watch the robot for 1 to 30 seconds; answers as soon as something happens to it | yes |
| `seen_cards()` | the cards the camera recognised, with the cell of each, where it was seen from, and when | yes |

**What the camera saw.** The camera names cards from a short list, never sends an image, and sees
in a cone in front of the robot (30 degrees each side, up to 120 cm: the simulator's values, TO BE
COMPLETED on the OpenMV N6), wider when farther; a box hides what is behind it. For each card it
gives an angle and a distance, which it guesses less well when the card is far (**Camera distance
error**). The robot only believes it while it stands still. Each card that comes into view becomes an
event (in `since_last_call`), and the gateway turns it into a cell, with the other cells it could be
in: "probably F3 (could also be F2, G3)". Seen again from closer, the estimate gets better.
`seen_cards` keeps that record, so it does not depend on the agent's memory; a card moved by hand
leaves it when the robot sees it gone. On the page, the shaded fan is what the camera can see, the
dots are where it thinks the cards are, and the dashed circles are what the gateway believes (what
the agent is told). The robot's own cell comes from counting moves: with **Wheels slip** on, it can
be wrong.

**What happened between two calls.** MCP works by questions and answers: an agent learns nothing
while it thinks. So every answer of the gateway carries `since_last_call`: what happened to the robot
since that caller's last call (something touched the bumper, came close in front, a move stopped, the
emergency button, Resume), with how many seconds ago. The robot itself remembers its last 16 events
(`EVENTS` in the protocol), so nothing is lost when nobody listens; an agent, a terminal and the page
each have their own count. `wait_for_event` lets an agent watch; it never makes it fast enough to
protect the robot: the reflexes do that.

**Events pushed to Claude Code.** With `ROBOT_CHANNEL=1` in the `env` of the
server in `.mcp.json`, the gateway also pushes each event as it happens, as a Claude Code *channel*:
Claude Code's own research preview, in interactive sessions only, started with
`MCP_PROTOCOL_NEGOTIATION=legacy ENABLE_TOOL_SEARCH=false claude --dangerously-load-development-channels server:robot`
([channels](https://code.claude.com/docs/en/channels)). Tried on 3 November 2026 with Claude Code 2.1.291 then 2.1.317, signed in with claude.ai: each event
started a turn with nobody typing, the line `← robot: Robot event, … s ago: …` on screen and an answer
from the model. Add `ENABLE_TOOL_SEARCH=false` to the command: by default Claude Code loads MCP tools
only on demand, and at the first event Haiku and Sonnet both answered they had no tool for the robot.
With it, Sonnet called `robot_state` at once, described the robot and moved nothing. Without network, Claude Code
only says "Channels are not currently available". Not tried with 2.1.303 nor with an API key. OpenCode 2.1.4 and Hermes v0.22.5 have no
such way: they read `since_last_call`.

- **Claude Code**: [.mcp.json](.mcp.json) declares the server. Claude Code asks you before each
  call; that question is the human in the loop.
- **Hermes Agent v0.22.5**: add [agents/hermes-robot.yaml](agents/hermes-robot.yaml) to its
  configuration. With `trust: untrusted`, Hermes asks a person before each move, and refuses every
  move where nobody can answer. Tried with a fake model, then with DeepSeek v4 Flash through
  OpenRouter (4 sessions, 0.0117 dollars): the model finds the robot's tools through Hermes's
  `tool_search`, calls `robot_state`, says so when a move is refused, and watches with
  `wait_for_event`. With `trust: full` (no approval, in the simulator only), it drove the robot
  by itself around a tree and a rock to a card, in 13 moves and 3 minutes. Hermes keeps the
  gateway running between two commands; the gateway follows the simulator if it restarts.
- **Any agent**: the result of each call is written to `.robot-sim/context.jsonl`, and the page
  shows it. This is exactly what entered the agent's context.

> [!WARNING]
> Each call sends the robot's state to the company that runs the model. Nothing about a person
> must go there.

---

## Check that everything works

```bash
/opt/workshop/venv/bin/python -I ci/check_sim.py
```

About two minutes, no model and no key. It plays each scenario of part 2 through MCP, as an agent
would, and reads in the simulator what really happened: 72 checks, all passed in the image
2026.10.45 on 3 November 2026 (container with 2 processors, no network).

---

## The protocol between the gateway and the robot

One line of text per command, one line per answer; the head of
[robot/firmware.py](robot/firmware.py) lists them all. The robot never moves backwards (its distance
sensor looks forwards only), moves one cell at most per command, and after an interrupted move
waits for an adult: the computer cannot release it.

---

## Limits to know

- **Simulator values.** Cells of 30 cm, 10 cm/s, a one-second watchdog, a sensor that sees a fence
  10 cm outside the mat (any closer, and the robot could not enter the cells of the edge): the real
  numbers are TO BE COMPLETED on the robot.
- **What the simulator cannot tell you**: how the real motors, wheels and sensors behave, the
  battery, the camera's real mistakes, the radio range, and whether children are safe around the
  robot.
- **The real robot**: the small file that gives `firmware.py` the board's sensors and motors is TO
  BE COMPLETED once the robot is built.
- **No real model has driven it**, and no group has used it.

---

## About

Texts: CC BY-SA 4.0. Code and configuration: EUPL-1.2. See [REUSE.toml](REUSE.toml).
