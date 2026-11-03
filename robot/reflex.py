"""The reflexes of the robot: what it does by itself, without asking the computer or the agent.

This file runs on the robot and in the simulator, unchanged. Change a number, press
"Reload reflex.py" in the simulator, and watch what changes before you try it on the real robot.

The numbers below are the simulator's. On the real robot they are TO BE COMPLETED: measure them.
The program of the robot (firmware.py) keeps its own limits: you can make the robot more careful
here, never less careful than those limits.
"""

# Stop when something is closer than this in front of the robot (centimetres).
STOP_DISTANCE_CM = 8

# The fastest the robot may go (centimetres per second).
MAX_SPEED_CM_S = 10

# While it moves, the computer says "I am still here" several times per second. When it has heard
# nothing for this long (milliseconds), the robot stops: the cable is loose, or the computer froze.
WATCHDOG_MS = 1000

# Below this battery level (percent), the robot refuses to move.
MIN_BATTERY = 20


def must_stop(distance_cm, battery):
    """Why the robot must stop now, or None to go on. Called many times per second while it moves.

    distance_cm: what the distance sensor sees in front, or None when nothing is in range (or
    when the robot is turning on the spot). battery: percent.
    """
    if distance_cm is not None and distance_cm < STOP_DISTANCE_CM:
        return "obstacle"
    if battery < MIN_BATTERY:
        return "battery"
    return None
