# What was tried, and what it showed

The [README](README.md) explains the robot and how to try it. This page keeps the trials behind its advice: which
models and which agents drove the simulated robot, when, how many times, and what happened.

Read every result with three limits in mind:

- **Few runs.** Most cases ran once to five times. A clear difference is a hint, not a proof.
- **The simulator only.** No real robot and no group of young people took part.
- **Earlier programs.** Several trials ran before the stack of this guide existed, with an earlier program that held
  the same tools and rules in one piece. Each section says so when it is the case.

---

## Which size of tools for the Strategist (6 November 2026)

**The question.** The Strategist's model can get big tools (go to a card), middle-sized ones (go to a cell, look
around: the route tools, used by default) or small ones (one step at a time). Which works, and what does each cost?

**How.** The five missions of the bench, one run per mission and per model, with Qwen3.8 Flash and DeepSeek v4 Flash
through OpenRouter. When there are two numbers, there is one per model. These runs used the loop the Strategist comes
from, not yet this stack.

| Tools given to the model | Missions done | Cost of a run (average) | Calls to the model (median) |
|---|---|---|---|
| none: the Pilot alone | 5 out of 5 (no model, one run per mission) | 0 $ | 0 |
| goal tools (`explore`, `go_to_card`) | 10 out of 10 | 0.0008 to 0.0017 $ | 4 |
| route tools, adventurous personality | 10 out of 10 | 0.006 to 0.013 $ | 16 to 20 |
| route tools, cautious personality | 8 out of 10, failures for lack of moves | 0.016 $ | 31 |
| step tools (the nine of part 7) | 9 out of 10 | 0.020 to 0.030 $ | 39 to 42 |

**Does the personality change the driving?** Yes, with both models, the way it should. The adventurous card looked
around 0.20 and 0.28 times per cell and took 45 and 11 hits from the ball; the cautious one looked 0.64 and 0.68
times per cell, took 7 and 2 hits, and ran out of moves twice (that bench gave 40 or 50 moves; this stack
sets no limit by default).

**No collision anywhere.** A ball touching the bumper is not one: the Reflex stops everything and an adult puts the
robot back.

**What the guide keeps.** The route tools are the default: the personality has a real effect on how the robot drives,
without every step going through the model.

---

## Interpreter: real models talk with a scripted group (7 November 2026)

**How.** One try each. A script played the group, in French. Two missions: find the barbecue; find dry grass.

- **DeepSeek v4 Flash through OpenRouter, for both Feurisson and the Strategist.** The two missions were done (the
  barbecue on F3; dry grass on C2 and D2), for 0.018 US dollars for two conversations. Feurisson waited for the
  group's yes every time. But it once gave no report after reading it, once told an old state without asking
  `mission_status`, and once stopped a mission without asking the group first.
- **Mistral Small 3.2 on Albert API, as Feurisson.** It started missions before the yes, then called a tool without a
  name, and Albert refused the rest of the conversation. **As the Strategist:** the same refused call again and again,
  and a card reported where it never was. Not usable for either.
- **`deepseek-v4-flash-0731` on Albert API, as the Strategist.** The two missions were done.

**What changed after.** The stack now refuses a report with a card the camera never placed there, and ends a mission
whose model sends the same refused call four times.

## Interpreter: what the filter changed (48 conversations)

Rewording Feurisson's instructions did not stop it from answering in the wrong language, nor from stopping a mission
without asking. So the filter was written (part 6 of the README). On 48 conversations with DeepSeek v4 Flash, played
by a script (the same 48 conversations, 198 turns, played without and with the filter):

- answers in the wrong language: from 25 to 0;
- missions stopped without asking the group: from 4 to 0;
- with the note on `mission_status`, answers that said nothing to the group: from 6 to 1.

---

## Agents of your own drive the robot (5 and 6 November 2026)

These trials used the nine step tools of part 7. All of them ran before the stack, with the earlier program.

**Two extra tools (5 November, Hermes).** With `look_around` and `face`, the models that wandered most found the card
more often. This is why each of them counts as one move.

**Time between two calls (5 November).** 6 seconds between two calls of an agent (median of the bench): by then, a rolling
ball has already gone more than a cell.

**`move` (6 November, Hermes and three cheap models).** 27 missions out of 30 with `move`, against 20 without it. The
rules about cards, ball lanes and waiting came after this bench.

**Hermes Agent v0.22.5.** With `trust: untrusted`, Hermes asks a person before each move and refuses every move when
nobody can answer. Tried with a fake model, then with DeepSeek v4 Flash through OpenRouter (4 sessions, 0.0117
dollars): the model found the robot's tools through Hermes's `tool_search`, called `robot_state`, said so when a move
was refused, and watched with `wait_for_event`. With `trust: full` (no approval, in the simulator only), it drove the
robot by itself around a tree and a rock to a card, in 13 moves and 3 minutes. Hermes kept `drive.py` running between
two commands, and followed the simulator when it restarted.

**Rules written by a group (5 November, Hermes).** With [agents/AGENTS.example.md](agents/AGENTS.example.md):

- DeepSeek v4 Flash found the card and stopped in front of it, looked around, let the ball go by: no collision;
- `qwen2.5:7b` on the laptop (Ollama, 6 GB graphics card) gave up after 7 calls;
- with OpenCode and the same small local models (`qwen2.5:7b`, `qwen3:1.7b`), the robot moved but did not find the
  card; nor with `qwen3:30b-a3b-instruct-2507` (18 GB) through OpenCode or Hermes, on the same laptop.

**Three coding agents, with only the robot's tools.** Claude Code (Sonnet 5.5), Codex (gpt-6.1-sol) and agy all found
the card. Only Codex waited to see the ball move away before it crossed. The other two crossed without looking: the
robot's own reflex stopped them, and their reports said they had waited.

**Five runs of each engine with each model** (Claude Code, OpenCode, Hermes): the model makes the main difference.
Sonnet 5.5 found the card in 4, 2 and 4 runs out of 5. DeepSeek v4 Flash never found it, in any engine. On the laptop,
`qwen3:8b` was too slow for a robot that moves in real time, and Hermes refuses a model with less than 64,000 tokens of
context.

**Hermes and nine models** (three runs each, then five runs of four ways to help the agent). Gemini 4 Argon found the
card 3 times out of 3, and Qwen3.8 Flash in most runs, for about 2 US cents a run. The two extra tools helped the
models that wandered most. Clearer rules did not help. GPT-6.1 Sol was the only model that crossed the ball's path
only after it saw the ball moving away; this is also why it often ran out of turns. The other models crossed without
looking: the robot's reflex is what protected it.

**What the guide keeps.** Read `.robot-sim/context.jsonl`, not only the agent's answer: an agent may say it waited
when it did not. And never count on the model for safety: the Reflex and the Referee do that.

---

## Events pushed to Claude Code (3 November 2026)

This trial ran before the stack, with the earlier program.

With `ROBOT_CHANNEL=1`, `drive.py` pushes each event as it happens, as a Claude Code *channel* (part 7 of the README).
Tried with Claude Code 2.1.291 then 2.1.317, signed in with claude.ai: each event started a turn with nobody typing,
with the line `← robot: Robot event, … s ago: …` on screen and an answer from the model. Without
`ENABLE_TOOL_SEARCH=false`, at the first event, Haiku and Sonnet both answered that they had no tool for the robot.
With it, Sonnet called `robot_state` at once, described the robot and moved nothing. Without network, Claude Code only
says "Channels are not currently available". Not tried with 2.1.303 nor with an API key. OpenCode 2.1.4 and Hermes
v0.22.5 have no such way: they read `since_last_call`.
