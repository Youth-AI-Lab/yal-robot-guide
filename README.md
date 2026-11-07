# A robot driven by an agent: the simulator

## Youth AI Lab - Erasmus+ project

Partners: L.A.B. (Aix-en-Provence, France), UAB (Barcelona, Spain), Perlatecnica (Naples, Italy),
Digitale Wolven (Antwerp, Belgium), CSC Vent des Îles (La Rochelle, France).

> [!WARNING]
> **A prototype, for facilitators.** Nothing here has run on a real robot, and no real model has
> driven it yet. Never use it with young people: the rules for that are not decided.

The young people of the project will build a robot (a STeaMi board and an OpenMV N6 camera) to give
their chatbot a body. Before the robot exists, this simulator lets you try the whole chain: the
program of the robot, and the stack of programs above it that decide what it does, down to the one
that is the only door to it.

```
 robot (or this simulator)          the stack (stack/), on a Raspberry Pi or on this computer
 ┌──────────────────────┐  text   ┌─────────┐   ┌──────────────┐   ┌───────┐   ┌────────────┐
 │ Reflex: stops, never │ ◄─────► │ Referee │ ◄►│ Cartographer │ ◄►│ Pilot │ ◄►│ Strategist │ ◄── mission
 │ waits for anyone     │  cable  │ the     │   │ the map of   │   │ the   │   │ a loop and │     (closed
 │ camera: a card label │         │ rules   │   │ the terrain  │   │ way   │   │ a model    │     format)
 └──────────────────────┘         └─────────┘   └──────────────┘   └───────┘   └────────────┘
```

Each tier of the stack talks only to its neighbours, in a language of its own (the head of each file
says which):

| Tier | What it does | File |
|---|---|---|
| Reflex | on the robot: stops by itself (bumper, obstacle, silent cable), moves, keeps its last events | [robot/firmware.py](robot/firmware.py), [robot/reflex.py](robot/reflex.py) |
| Referee | the only door to the robot: judges each command against the rules written by the adults, refuses, counts the moves, keeps a journal | [stack/referee.py](stack/referee.py), [stack/rules.json](stack/rules.json) |
| Cartographer | the map of the terrain (where the robot probably is, the cards, the balls, the cells seen free), and the translation of "go east" into "slide to the right" | [stack/cartographer.py](stack/cartographer.py) |
| Pilot | code without a model: how to reach the goal (the way, exploring, waiting for an adult) | [stack/pilot.py](stack/pilot.py) |
| Strategist | a loop and a model: which tool next, following the group's personality; ends each mission with a closed report | [stack/strategist.py](stack/strategist.py) |
| Interpreter | the chatbot that turns the group's words into a mission, and the report back into words (Open WebUI, part 6); [stack/mission.py](stack/mission.py) stands in for it in a terminal | [interpreter/feurisson.md](interpreter/feurisson.md), [stack/interpreter_link.py](stack/interpreter_link.py) |

Three rules hold the design together:

- **The robot protects itself.** It stops in front of an obstacle, when the cable comes loose, and
  when someone presses its emergency button, without asking the computer
  ([robot/reflex.py](robot/reflex.py), [robot/firmware.py](robot/firmware.py)).
- **The Referee is the only door.** Four cells at most per move (a move counts one, however many
  cells), never off the mat, never onto a card, backwards or sideways only into cells its sensor saw
  free, a number of moves per session. No personality and no model can loosen these rules
  ([stack/rules.json](stack/rules.json), [mat.json](mat.json)). The rules also say where the robot
  starts (`start`: a cell and a way to face, only the way it faces such as `{"facing": "north"}` for
  the top of the mat, or `"unknown"`) and which cards are landmarks on a known
  cell (`landmarks`). The mat always lies with north at the top (`mat_north_up`): the robot's compass
  gives the way it faces. With an unknown start, the Cartographer counts from where the robot started
  and rules places out with the compass, each reading of the distance sensor (the mat has a border it
  sees) and each landmark the camera recognises; one look around is usually enough. Meanwhile the
  robot only turns, and steps forwards where the way is seen free.
- **No image ever leaves the robot.** The camera gives the name of a card from a short list.

The simulator replaces only the physical part. The program of the robot runs in it unchanged, and
the Referee talks to it through a pseudo-terminal, as it will talk to the robot through its cable.

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

## Part 2 - Drive it, through the stack

The robot moves only through the stack, down to the Referee: the same door the Strategist uses, with the
same rules. Two ways to drive it by hand:

- **On the page**, the **Drive** panel. **North**, **West**, **South**, **East** and the arrow keys of
  the keyboard go towards a side of the mat: the robot turns as needed, then moves one cell (as the
  Cartographer believes it faces). **↑ Forward**, **↺ Left**, **↻ Right** are the moves an agent has, one at
  a time. **State**, **Light**, **Seen cards** and **Wait for an event (10 s)** too. Each button runs the stack, as
  the command below would.
- **In a second terminal**:

  ```bash
  python3 -I stack/drive.py state
  python3 -I stack/drive.py forward
  python3 -I stack/drive.py right
  python3 -I stack/drive.py around
  python3 -I stack/drive.py face east
  python3 -I stack/drive.py move north 3 card_ahead
  python3 -I stack/drive.py light green
  ```

Each answer is what an agent would receive, word for word. On the page, **On the wire** shows the
lines between the computer and the robot, and **What the agent received** shows these answers, with
who sent the order: an agent, a terminal or the Drive buttons. One order at a time: while one runs,
any other is refused, wherever it comes from.

You can drag with the mouse everything that lies on the mat: the boxes (obstacles, brown squares,
**Add a box** adds one, a double-click removes one), the cards (they land on a cell) and the ball
(with its whole path). Not the robot: the Cartographer counts its moves to know where it is, so a robot
moved by hand would be lost. Press **Start again** to put it back on its start cell.

Things to try:

- Drag a box onto the cell in front of the robot (the dashed square), then **Forward**: refused,
  "something is in front of the robot".
- **Forward** and, while it moves, drag a box right in front of it: the robot stops its motors and
  waits up to 3 seconds (`OBSTACLE_WAIT_MS` in `robot/reflex.py`). Take the box away in time: it
  finishes its cell. Leave it: it gives up, between two cells, and the Cartographer refuses every move,
  since the position is unknown. Press **An adult puts it back and presses Resume**, then
  **Forward** again. With the box there before the move, it waits, then refuses: it never moved, so
  no adult is needed.
- **Forward** and, while it moves, **Unplug the cable**: the robot stops after one second (its
  watchdog). The Referee waits, then says the move was stopped.
- **Emergency stop**, then **Forward**: refused.
- Turn towards the edge and **Forward**: refused, the next cell is outside the mat.
- Drag a card onto the cell in front of the robot, then **State**: `card_ahead` names it.
- `move north 3 card_ahead`: the robot slides up to three cells and stops on the cell in front of
  the first card it meets. Then `move east`: refused, the robot only sees in front of it. `face east`,
  `face north`, `move east` again: now the distance sensor has seen the cell free, it slides there
  without turning.
- Put the robot on the path of the ball (column E) and wait. When the ball comes close in front, the
  light turns red (the warning reflex); when it touches the robot, it bounces off the bumper. The
  robot does not move away: standing still is its safe state, and moving is a decision. While it
  moves, the ball stops it (distance sensor in front, bumper on the sides).

> [!NOTE]
> The Cartographer counts the moves to know where the robot is: it never sees the real cell. Set
> **Wheels slip** to 10 %, drive a few cells, and compare **Real cell** on the page with the `cell`
> the Cartographer gives.

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

## Part 5 - Give it a mission

The Strategist takes a mission in a closed format: a list of steps, each one an action from a short
list with one argument from a short list, and a personality. It never receives the group's words: it
builds the sentences its model reads from these fields alone, and the model ends the mission with
`report`, whose fields are closed too. Until the Interpreter (the chatbot that turns the group's words
into a mission) is written, [stack/mission.py](stack/mission.py) stands in for it.

| Step | Argument | What the robot does |
|---|---|---|
| `reach_card` | a card of [mat.json](mat.json) | stops right in front of it, facing it |
| `set_light` | `off`, `red`, `orange`, `green`, `blue` | sets its light |
| `find_card` | a card | finds out whether it is on the mat, and on which cell |
| `survey` | a card | finds every card of that kind, with its cell |

1. Without a model, the Pilot alone does each step (no network, no cost). With the simulator running,
   in a second terminal:

   ```bash
   python3 -I stack/mission.py reach_card:barbecue set_light:green --model none
   ```

   You should see its report, on one line, with `"outcome": "done"` and
   `"cards": [{"card": "barbecue", "cell": "F3"}]`.

2. With a scripted model, which checks the loop without any network: `--model fake`.

3. With a real model, through OpenRouter. The key is read on stdin, never on the command line, and
   `--cap` sets the most the mission may cost, in US dollars:

   ```bash
   read -rs KEY && printf %s "$KEY" | python3 -I stack/mission.py find_card:barbecue --model qwen-flash --cap 0.10; unset KEY
   ```

   French public servants (teachers included) can use Albert API of the DINUM instead, with their
   Albert key: `--model albert:deepseek-v4-flash` (or another of its models). No price; the content
   is not kept; hosted in France.

   > [!WARNING]
   > Each call sends the mission and the robot's state to the company or the administration that runs
   > the model. Nothing about a person must go there.

4. A personality: `--personality cautious` or `--personality adventurous`, two compositions of
   bricks from [stack/personality/bricks.json](stack/personality/bricks.json). A brick is either a
   setting of the Pilot (code: always applied, such as "2 cells at most in one move") or an
   instruction to the Strategist's model (text: often followed, sometimes forgotten). No brick can
   loosen the Referee's rules.

5. In another terminal, `python3 -I stack/mission.py pause`, then `resume`, or `stop`. The loop
   handles it without asking the model: the robot stops on the next cell, as soon as possible (the
   Pilot asks it, down to the Reflex: `HOLD` in the protocol), and no adult is needed.

What to read afterwards, in `.robot-sim/`: `missions/<time>/` (the mission, every message the model
received and sent, the report, what it cost) and `referee.jsonl` (every command the Referee sent or
refused, with the rule).

The Strategist offers its model one size of tools: the route tools by default (`go_to_cell`,
`look_around`, `face`, `wait_for_event`, `seen_cards`, `set_light`, `robot_state`), `--tools goal`
(`explore`, `go_to_card`) or `--tools step` (the nine tools of part 7). On 6 November 2026, with the
loop this one comes from, one run per mission with Qwen3.8 Flash and DeepSeek v4 Flash: with the route
tools, the adventurous card looked 0.20 to 0.28 times per cell and the cautious one 0.64 to 0.68, and
the cautious one failed twice in ten runs, for lack of moves. Not yet run with this stack.

---

## Part 6 - Talk to it: the Interpreter (not tried with a group)

The Interpreter is a chatbot: Feurisson, the voice of the robot. It talks with the group in the
group's words, turns their request into a mission in the closed format, says it back, and starts it
only after the group says yes. It reaches the stack through one door,
[stack/interpreter_link.py](stack/interpreter_link.py): an MCP server over HTTP with a token, six
tools (the closed lists, start, pause, resume, stop, the mission's state and report), none of which
drives the robot. Its instructions are [interpreter/feurisson.md](interpreter/feurisson.md).

1. With the simulator running, in a second terminal, start the link. The token is a secret of at
   least 32 characters; keep it for Open WebUI:

   ```bash
   export MCP_TOKEN=$(python3 -c "import secrets; print(secrets.token_hex(24))")
   /opt/workshop/venv/bin/python -I stack/interpreter_link.py --model none --allowed-host 127.0.0.1:8790
   ```

   You should see `READY robot-mission: http://127.0.0.1:8790/mcp, Strategist none with the route
   tools`. `--model` is the Strategist's model, as in part 5 (`none`, `fake`, `albert:<model>`, a
   model of OpenRouter; a key on stdin).

2. In Open WebUI, declare the link as a tool server of type MCP (Streamable HTTP), its address
   ending in `/mcp`, authentication "bearer" with the token. From the environment, as the checks of
   this guide do:

   ```json
   TOOL_SERVER_CONNECTIONS=[{"type": "mcp", "url": "http://<this computer>:8790/mcp", "path": "",
     "auth_type": "bearer", "key": "<the token>", "config": {"enable": true},
     "info": {"id": "robot-mission", "name": "Robot mission"}}]
   ```

   Where to do it on screen: TO BE COMPLETED (not looked at for this guide).

3. Make a chatbot: the text of [interpreter/feurisson.md](interpreter/feurisson.md) as its System
   Prompt, the tool server "Robot mission" switched on, **Function Calling: Native**.

4. Its model, through an OpenAI-compatible connection of Open WebUI:
   - **Albert API**, for French public servants (teachers included, through ProConnect): address
     `https://albert.api.etalab.gouv.fr/v1`, your Albert key. No price; the content of the messages is
     not kept (the date, the account, the model, the number of tokens and the IP address are, for 24
     months); hosted in France. On 7 November 2026, `deepseek-v4-flash`, `gemma-4-31b-it`,
     `gpt-oss-120b`, `mistral-small-3-2-24b-instruct-2506` and `ministral-3-8b-instruct-2512` all
     called a tool correctly; the list changes (two of them leave on 1 December 2026).
   - **OpenRouter**, for the partners, as in the workshops.

> [!WARNING]
> Everything the group types goes to the company or the administration that runs the Interpreter's
> model. Nothing about a person must go there, and nothing of this is ready for a group of young
> people.

---

## Part 7 - Connect an agent of your own (not tested with a real model)

[stack/drive.py](stack/drive.py) is an MCP server: the Pilot's nine step tools, which go down the
whole stack, for an agent of your own instead of the Strategist:

| Tool | What it does | Read only |
|---|---|---|
| `robot_state()` | distance in front, card in front, battery, buttons, the cell where the Cartographer thinks the robot is | yes |
| `step_forward()` | one cell forwards, then stop | no |
| `move(direction, cells, stop_if)` | slide 1 to 4 cells `north`, `east`, `south` or `west` without turning; stops on a cell when `stop_if` is met: `card_ahead`, `ball_seen`, `new_card`. One move | no |
| `turn(direction)` | a quarter turn, `left` or `right` | no |
| `look_around()` | a full turn on the spot, a pause in each direction; says what the camera saw in each one. One move | no |
| `face(direction)` | turn on the spot towards `north`, `east`, `south` or `west`. One move, or none if it already faces it | no |
| `set_light(color)` | `off`, `red`, `orange`, `green` or `blue` | no |
| `wait_for_event(seconds)` | watch the robot for 1 to 30 seconds; answers as soon as something happens to it | yes |
| `seen_cards()` | the cards the camera recognised, with the cell of each, where it was seen from, and when; the cells the camera has not looked at yet | yes |

`look_around` and `face` count as one move each, so that an agent spends its moves on going
somewhere, not on counting quarter turns. Tried on 5 November 2026 with Hermes: with these two
tools, the models that wandered most found the card more often.

**`move`: several cells, and conditions.** The robot has mecanum wheels: it slides forwards,
backwards or to a side without turning, and its camera keeps looking the way it faces. One `move`
counts as one move, however many cells (1 to 4). The robot itself checks the conditions, on each
cell it reaches, and stops there: the agent answers seconds later (6 s between two calls, median of
the bench of 5 November), when a ball has already rolled more than a cell. The robot never drives onto
a card it knows: forwards it always stops in front of one; every slide stops before a cell where the
Cartographer has placed a card for sure, and a slide sideways or backwards before any card the camera has
placed, even from far; `step_forward` and a slide forwards refuse a cell with a card, and a cell where a
ball was seen in the last 10 seconds (or next to one that was coming closer). Its distance sensor and
camera only look forwards: backwards or sideways, the Referee lets it slide only into cells the
sensor saw free in the last two minutes (`face` that way once, or `look_around`; the cells it has just
been through count too), never into the lane of a ball (the row or column where one was seen
rolling; seen from far, a lane may cover two), never near a ball seen in the last 10 seconds, and
the robot goes slower there. The sensor's
beam is narrower than the robot (TO BE COMPLETED on the real one): a small thing near the side of a
cell can be missed, and then the bumper stops the robot. How true a sideways slide is on the real
wheels is TO BE COMPLETED (mecanum wheels slip more sideways). Tried on 6 November with Hermes and
three cheap models: 27 missions out of 30, against 20 without `move`; the rules about cards, lanes
and waiting came after that bench.

**What the camera saw.** The camera names cards from a short list, never sends an image, and sees
in a cone in front of the robot (30 degrees each side, up to 120 cm: the simulator's values, TO BE
COMPLETED on the OpenMV N6), wider when farther; a box hides what is behind it. For each card it
gives an angle and a distance, which it guesses less well when the card is far (**Camera distance
error**). The robot only believes it while it stands still. Each card that comes into view becomes an
event (in `since_last_call`), and the Cartographer turns it into a cell, with the other cells it could be
in: "probably F3 (could also be F2, G3)". Seen again from closer, the estimate gets better.
`seen_cards` keeps that record, so it does not depend on the agent's memory; a card moved by hand
leaves it when the robot sees it gone. It also keeps which cells the camera has looked at
(`camera_coverage`): a cell counts once the robot stood still with it in the camera's cone, unless the
line to it crosses a cell where the distance sensor met something, and a cell the robot stood on
counts too. A cell where the sensor met something at two moments 10 s apart is set apart: something
stands there (a box; a ball rolls on). When no cell is left, the agent hears it once: the cards on the
mat are the ones the camera saw. The Cartographer knows boxes only through the distance sensor: a box it
never met can still hide a cell, which the agent is not told to guess about. The camera also sees the ball: twice a second, the robot
compares two sightings and tells when it starts coming closer, going away, or leaves its sight, and
the Cartographer says where it is and which way it goes ("probably E3, moving south, getting closer to
the robot"); `robot_state` gives the last sighting. On the page, the shaded fan is what the camera can see, the
dots are where it thinks the cards are, and the dashed circles are what the Cartographer believes (what
the agent is told). The robot's own cell comes from counting moves: with **Wheels slip** on, it can
be wrong.

**What happened between two calls.** MCP works by questions and answers: an agent learns nothing
while it thinks. So every answer of the stack carries `since_last_call`: what happened to the robot
since that caller's last call (something touched the bumper, came close in front, a move stopped, the
emergency button, Resume), with how many seconds ago. The robot itself remembers its last 16 events
(`EVENTS` in the protocol), so nothing is lost when nobody listens; an agent, a terminal and the page
each have their own count. `wait_for_event` lets an agent watch; it never makes it fast enough to
protect the robot: the reflexes do that.

**Events pushed to Claude Code.** With `ROBOT_CHANNEL=1` in the `env` of the
server in `.mcp.json`, drive.py also pushes each event as it happens, as a Claude Code *channel*:
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
  by itself around a tree and a rock to a card, in 13 moves and 3 minutes. Hermes keeps
  drive.py running between two commands; it follows the simulator if it restarts. These trials ran
  with the gateway that the stack replaced (the same tools, the same rules, in one program).
- **Any agent**: the result of each call is written to `.robot-sim/context.jsonl`, and the page
  shows it. This is exactly what entered the agent's context.
- **Rules for the agent**: [agents/AGENTS.example.md](agents/AGENTS.example.md) is an example of what
  a group could write (look around before moving, cross the ball's path only when it moves away,
  wait when the robot asks for an adult). Name it `AGENTS.md` in the folder where you start the
  agent. Tried on 5 November 2026 with Hermes: with DeepSeek v4 Flash, the robot found the card and
  stopped in front of it, looked around, let the ball go by, no collision; with `qwen2.5:7b` on the
  laptop (Ollama, 6 GB card), it gave up after 7 calls. With OpenCode and the same small local models
  (`qwen2.5:7b`, `qwen3:1.7b`), the robot moved but did not find the card; nor with
  `qwen3:30b-a3b-instruct-2507` (18 GB) through OpenCode or Hermes, on the same laptop. The same task with
  three coding agents that had only the robot's tools: Claude Code (Sonnet 5.5), Codex (gpt-6.1-sol) and
  agy all found the card. Only Codex waited to see the ball move away before it crossed. The other two
  crossed without looking, the robot's own reflex stopped them, and their reports said they had waited.
  Read `.robot-sim/context.jsonl`, not only the agent's answer. Five runs of each engine (Claude Code,
  OpenCode, Hermes) with each model showed that the model makes the main difference. Sonnet 5.5 found the
  card in 4, 2 and 4 runs out of 5. DeepSeek v4 Flash never found it, in any engine. On the laptop,
  `qwen3:8b` was too slow for a robot that moves in real time, and Hermes refuses a model with less than
  64,000 tokens of context. With Hermes and nine models (three runs each, then five runs of four
  ways to help the agent), Gemini 4 Argon found the card 3 times out of 3 and Qwen3.8 Flash in most
  runs, for about 2 US cents a run. Two extra tools, one to look all around and one to face north,
  east, south or west, helped the models that wandered most. Clearer rules did not help.
  GPT-6.1 Sol was the only model that crossed the ball's path only after it saw the ball moving
  away; this is also why it often ran out of turns. The other models crossed without looking:
  the robot's reflex is what protected it.
- **One move at a time.** A move sent while another one runs is refused ("send one move at a time"):
  a model that sends several moves at once would otherwise see them run in an order nobody chose.

> [!WARNING]
> Each call sends the robot's state to the company that runs the model. Nothing about a person
> must go there.

---

## Check that everything works

```bash
/opt/workshop/venv/bin/python -I ci/check_sim.py
```

About eight minutes, no model and no key. It plays each scenario of part 2 through MCP, as an agent
would, then gives missions to the Strategist with no model or a scripted one, then through the
Interpreter's link over HTTP, as Open WebUI would, and reads in the simulator what really happened:
164 checks, all passed in the image 2026.10.45 on 7 November 2026 (container with 2 processors, no
network). A real Open WebUI (0.12.4) with a fake model, then with Albert's `gemma-4-31b-it` as the
Interpreter, was tried apart, not in these checks.

---

## The protocol between the Referee and the robot

One line of text per command, one line per answer; the head of
[robot/firmware.py](robot/firmware.py) lists them all. The robot moves forwards, or slides backwards
or sideways without turning (more slowly: its distance sensor and camera look forwards only), four
cells at most per command, and always stops on a cell. After a move interrupted between two cells it
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
