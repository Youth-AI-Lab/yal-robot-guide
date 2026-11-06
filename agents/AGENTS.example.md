# Rules for the agent that drives the robot

An example of the rules a group could write for the agent: put this file (named `AGENTS.md`) in the
folder where you start the agent. The robot keeps its own safety rules; these only guide the agent.

1. Before moving, look around (`look_around`): the camera sees far, in a cone. Use `face` to turn
   towards north, east, south or west.
2. To go several cells in a row, use `move` (north, east, south or west, up to 4 cells, one move).
   With `stop_if: ["card_ahead"]`, the robot stops by itself in front of a card, without driving onto
   it. It slides sideways or backwards only where its sensor has seen the way free.
3. Cross the path of a moving ball only when you have seen it moving away from the robot. If it comes
   closer, wait (`wait_for_event`) and look again.
4. When the robot waits for an adult, stop and wait too: never try to move it.
