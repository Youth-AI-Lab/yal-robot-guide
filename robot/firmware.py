"""The program of the robot. It runs in the simulator, and is written to run unchanged on the robot.

It is written in the part of Python that MicroPython understands (no import, no f-string, no type
annotation), because the robot's board runs MicroPython. On the real board, a small file gives it
the HAL below; that file is TO BE COMPLETED once the robot is built.

The robot reads one command per line from the computer (the gateway) and answers one line. It
moves one cell at most per command. Its reflexes (reflex.py) can stop it at any time, without
asking the computer. Nothing else is understood.

    HELLO          ->  OK HELLO robot <protocol>
    PING           ->  OK PONG              (the computer says it is still there)
    STATE          ->  OK STATE dist=<cm|none> card=<label|none> battery=<percent>
                       estop=<0|1> wait_adult=<0|1> moving=<0|1> last_stop=<reason>
    MOVE F         ->  OK MOVE F, then EVT DONE F, or EVT STOP <reason>
    TURN L, TURN R ->  OK TURN L, then EVT DONE L, or EVT STOP <reason>
    LED <color>    ->  OK LED <color>
    STOP           ->  OK STOP
    anything else  ->  ERR <reason>

The robot never moves backwards: its distance sensor only looks forwards.

After a move is interrupted (obstacle, watchdog, emergency button, STOP), the robot is somewhere
between two cells. It refuses to move until an adult puts it back on the cell where the move
started and presses the Resume button on the robot (EVT RESUMED). Nothing from the computer can
do that.

What the program needs from the board (hal):
    ticks_ms(), ticks_diff(new, old)   a clock in milliseconds
    read_lines(), write_line(text)     the link to the computer
    drive(speed_cm_s, turn_deg_s)      both wheels: forwards speed, and turning speed (+ = right)
    odometry()                         (centimetres, degrees) counted by the wheels since start
    distance_cm()                      the distance sensor in front: a number, or None
    battery()                          percent
    button(name)                       "stop" or "resume": True while pressed
    camera_label()                     the last card the camera recognised, or None
    led(color)                         one of COLORS
"""

PROTOCOL = 1

# The length of one cell of the mat (centimetres): the same number as "cell_cm" in mat.json.
CELL_CM = 30

# Limits that reflex.py cannot loosen.
HARD_MAX_SPEED_CM_S = 20
HARD_MIN_DISTANCE_CM = 3
TURN_DEG_S = 90

COLORS = ("off", "red", "orange", "green", "blue")


class Firmware:
    def __init__(self, hal, reflex):
        self.hal = hal
        self.reflex = reflex
        self.motion = None  # None, or [kind, start] with kind "F", "L" or "R"
        self.estop = False
        self.wait_adult = False
        self.last_stop = "none"
        self.last_heard = hal.ticks_ms()

    def say(self, text):
        self.hal.write_line(text)

    def tick(self):
        """Called many times per second (100 times in the simulator)."""
        hal = self.hal
        now = hal.ticks_ms()
        for line in hal.read_lines():
            self.last_heard = now
            self.answer(line.strip())

        if hal.button("stop") and not self.estop:
            self.estop = True
            self.halt("estop")
        if hal.button("resume") and not hal.button("stop") and (self.estop or self.wait_adult):
            self.estop = False
            self.wait_adult = False
            self.say("EVT RESUMED")

        if self.motion is None:
            return
        kind = self.motion[0]
        if hal.ticks_diff(now, self.last_heard) > self.reflex.WATCHDOG_MS:
            self.halt("watchdog")
            return
        why = self.stop_reason(kind)
        if why:
            self.halt(why)
            return
        self.steer()

    def stop_reason(self, kind):
        distance = None
        if kind == "F":
            distance = self.hal.distance_cm()
            if distance is not None and distance < HARD_MIN_DISTANCE_CM:
                return "obstacle"
        return self.reflex.must_stop(distance, self.hal.battery())

    def steer(self):
        """Drive towards the end of the move, slower near the end, and stop there."""
        cm, deg = self.hal.odometry()
        kind, start = self.motion
        if kind == "F":
            left = CELL_CM - (cm - start)
            if left <= 0:
                self.finish()
                return
            top = min(self.reflex.MAX_SPEED_CM_S, HARD_MAX_SPEED_CM_S)
            self.hal.drive(max(2, min(top, left * 3)), 0)
        else:
            left = 90 - abs(deg - start)
            if left <= 0:
                self.finish()
                return
            rate = max(10, min(TURN_DEG_S, left * 4))
            if kind == "L":
                rate = -rate
            self.hal.drive(0, rate)

    def finish(self):
        self.hal.drive(0, 0)
        kind = self.motion[0]
        self.motion = None
        self.say("EVT DONE " + kind)

    def halt(self, why):
        self.hal.drive(0, 0)
        self.last_stop = why
        if self.motion is not None:
            self.motion = None
            self.wait_adult = True
            self.say("EVT STOP " + why)

    def refusal(self, kind):
        if self.estop:
            return "estop"
        if self.wait_adult:
            return "wait_adult"
        if self.motion is not None:
            return "busy"
        why = self.stop_reason(kind)
        if why:
            return why
        if kind == "F":
            # The whole cell must be free, plus the stopping distance.
            distance = self.hal.distance_cm()
            if distance is not None and distance < CELL_CM + self.reflex.STOP_DISTANCE_CM:
                return "obstacle"
        return None

    def answer(self, command):
        parts = command.split()
        if not parts:
            return
        word = parts[0]
        if word == "HELLO" and len(parts) == 1:
            self.say("OK HELLO robot %d" % PROTOCOL)
        elif word == "PING" and len(parts) == 1:
            self.say("OK PONG")
        elif word == "STATE" and len(parts) == 1:
            self.say(self.state_line())
        elif word == "STOP" and len(parts) == 1:
            self.halt("command")
            self.say("OK STOP")
        elif word == "LED" and len(parts) == 2 and parts[1] in COLORS:
            self.hal.led(parts[1])
            self.say("OK LED " + parts[1])
        elif (word == "MOVE" and parts[1:] == ["F"]) or (word == "TURN" and parts[1:] in (["L"], ["R"])):
            kind = parts[1]
            why = self.refusal(kind)
            if why:
                self.say("ERR " + why)
                return
            cm, deg = self.hal.odometry()
            self.motion = [kind, cm if kind == "F" else deg]
            self.say("OK " + word + " " + kind)
            self.steer()
        else:
            self.say("ERR unknown")

    def state_line(self):
        distance = self.hal.distance_cm()
        card = self.hal.camera_label()
        return "OK STATE dist=%s card=%s battery=%d estop=%d wait_adult=%d moving=%d last_stop=%s" % (
            "none" if distance is None else str(int(distance)),
            card or "none",
            int(self.hal.battery()),
            1 if self.estop else 0,
            1 if self.wait_adult else 0,
            0 if self.motion is None else 1,
            self.last_stop,
        )
