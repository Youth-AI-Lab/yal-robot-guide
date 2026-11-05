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
                       estop=<0|1> wait_adult=<0|1> moving=<0|1> last_stop=<reason> bumper=<0|1>
                       warning=<0|1> resumes=<number of presses on Resume since the robot started>
                       pose=<number of the robot's place: +1 after each move, stop or Resume>
    MOVE F         ->  OK MOVE F, then EVT DONE F, EVT STOP <reason> (stopped on the way: an adult must
                       come), or EVT REFUSED <reason> (it waited, the way stayed blocked, it never moved)
    TURN L, TURN R ->  OK TURN L, then EVT DONE L, or EVT STOP <reason>
    LED <color>    ->  OK LED <color>
    STOP           ->  OK STOP
    EVENTS <n>     ->  OK EVENTS now=<ms> next=<id> <id>:<ms>:<event> ...  (the remembered events after n)
    anything else  ->  ERR <reason>
and, without being asked:
    EVT BUMP       something touched the bumper (a ball, a foot), moving or not
    EVT NEAR       standing still, something came close in front: the light warns (reflex.warn)
    EVT SEEN:<card>:<angle>:<cm>:<pose>  standing still, the camera recognised this card, this many
                   degrees to the right (negative: to the left) and this far from the robot's centre
    EVT RESUMED    an adult pressed Resume on the robot
When something is in its way during a forward move, the robot stops its motors and waits up to
reflex.OBSTACLE_WAIT_MS for the way to clear (PAUSE, then GO_ON), then finishes the same move.

The robot also remembers its last 32 events (those, ESTOP, STOP:<reason> when a move stops, PAUSE and
GO_ON, and
GONE:<card>:<angle>:<cm>:<pose> when a card it saw leaves its view while it stands still), numbered,
with the time of its clock: a computer that was not listening asks EVENTS. The camera is only
believed while the robot stands still: during a move or a turn, nothing it sees is noted.

The robot never moves backwards: its distance sensor only looks forwards. Its bumper goes all
around it: when something touches it, a move stops at once, whatever reflex.py says.

After a move is interrupted (obstacle, watchdog, emergency button, STOP), the robot is somewhere
between two cells. It refuses to move until an adult puts it back on the cell where the move
started and presses the Resume button on the robot (EVT RESUMED, and "resumes" in STATE goes up
by one). Nothing from the computer can do that.

What the program needs from the board (hal):
    ticks_ms(), ticks_diff(new, old)   a clock in milliseconds
    read_lines(), write_line(text)     the link to the computer
    drive(speed_cm_s, turn_deg_s)      both wheels: forwards speed, and turning speed (+ = right)
    odometry()                         (centimetres, degrees) counted by the wheels since start
    distance_cm()                      the distance sensor in front: a number, or None
    battery()                          percent
    button(name)                       "stop" or "resume": True while pressed
    bumper()                           True while something touches the bumper
    camera_view()                      the cards the camera recognises now: a list of
                                       [card, angle in degrees (+ = right), distance in cm]
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
EVENTS_KEPT = 32
# The camera's answer must stay the same this long before the robot believes it (milliseconds). The
# simulator's value; on the real robot, the camera answers 1 to 2 s late (RobotIA): TO BE COMPLETED.
CAMERA_STABLE_MS = 200


class Firmware:
    def __init__(self, hal, reflex):
        self.hal = hal
        self.reflex = reflex
        self.motion = None  # None, or [kind, start] with kind "F", "L" or "R"
        self.paused_since = None  # while it waits for the way to clear
        self.estop = False
        self.wait_adult = False
        self.last_stop = "none"
        self.bumped = False
        self.resumes = 0
        self.resume_held = False
        self.light = "off"  # the colour asked by the computer
        self.warning = False
        self.events = []  # [number, time in ms, name], the last EVENTS_KEPT
        self.next_event = 1
        self.pose = 0  # +1 each time the robot may be somewhere else: a move ends, stops, or Resume
        self.view = []  # what the camera shows, once believed: [[card, angle, cm], ...]
        self.view_key = None
        self.candidate = None
        self.candidate_since = 0
        self.last_heard = hal.ticks_ms()

    def say(self, text):
        self.hal.write_line(text)

    def remember(self, name):
        self.events.append([self.next_event, self.hal.ticks_ms(), name])
        self.next_event += 1
        if len(self.events) > EVENTS_KEPT:
            self.events.pop(0)

    def note(self, name):
        """An event: remembered, and said at once to whoever listens."""
        self.remember(name)
        self.say("EVT " + name)

    def tick(self):
        """Called many times per second (100 times in the simulator)."""
        hal = self.hal
        now = hal.ticks_ms()
        for line in hal.read_lines():
            self.last_heard = now
            self.answer(line.strip())

        if hal.button("stop") and not self.estop:
            self.estop = True
            self.remember("ESTOP")
            self.halt("estop")
        touched = hal.bumper()
        if touched and not self.bumped:
            self.note("BUMP")
            self.halt("bump")
        self.bumped = touched
        resume = hal.button("resume") and not hal.button("stop")
        if resume and not self.resume_held:
            # Counted at each press, even when the robot was not waiting: the computer compares the
            # number, it does not need to catch the message.
            self.estop = False
            self.wait_adult = False
            self.resumes += 1
            self.pose += 1  # the adult put it back: a new place
            self.view, self.view_key = [], None
            self.note("RESUMED")
        self.resume_held = resume

        if self.motion is None:
            self.warn()
            self.look(now)
            return
        kind = self.motion[0]
        if hal.ticks_diff(now, self.last_heard) > self.reflex.WATCHDOG_MS:
            self.halt("watchdog")
            return
        why = self.stop_reason(kind)
        if why == "obstacle" and not self.hal.bumper():
            # Something in the way, not touching: wait for it to go away (a ball rolling past).
            self.hal.drive(0, 0)
            if self.paused_since is None:
                self.paused_since = now
                self.remember("PAUSE")
            elif hal.ticks_diff(now, self.paused_since) > getattr(self.reflex, "OBSTACLE_WAIT_MS", 0):
                self.give_up("obstacle")
            return
        if why:
            self.halt(why)
            return
        if self.paused_since is not None:
            self.paused_since = None
            self.remember("GO_ON")
        self.steer()

    def warn(self):
        """Standing still: the light warns when something is close. Only the light."""
        color = None
        if hasattr(self.reflex, "warn"):
            color = self.reflex.warn(self.hal.distance_cm())
        if color in COLORS:
            if not self.warning:
                self.warning = True
                self.note("NEAR")
            self.hal.led(color)
        elif self.warning:
            self.warning = False
            self.hal.led(self.light)

    def look(self, now):
        """Standing still: once the camera's view is stable, note each card that came into it, and
        each card that left it (moved by hand, hidden by a box)."""
        view = self.hal.camera_view()
        key = repr(view)
        if key != self.candidate:
            self.candidate, self.candidate_since = key, now
            return
        if key == self.view_key or self.hal.ticks_diff(now, self.candidate_since) < CAMERA_STABLE_MS:
            return
        left = list(self.view)
        self.view, self.view_key = view, key
        for card, angle, cm in view:
            if [card, angle, cm] in left:
                left.remove([card, angle, cm])
            else:
                self.note("SEEN:%s:%d:%d:%d" % (card, angle, cm, self.pose))
        for card, angle, cm in left:
            self.remember("GONE:%s:%d:%d:%d" % (card, angle, cm, self.pose))

    def card_in_front(self):
        """The nearest card just in front (the cell ahead), from what the camera shows."""
        best = None
        for card, angle, cm in self.hal.camera_view():
            if -15 <= angle <= 15 and cm <= CELL_CM * 3 // 2 and (best is None or cm < best[1]):
                best = [card, cm]
        return None if best is None else best[0]

    def stop_reason(self, kind):
        """Why the robot cannot go on now, or None. "obstacle": something in the way of the rest of the
        cell (closer than what is left to go, plus the stopping distance)."""
        distance = None
        if kind == "F":
            distance = self.hal.distance_cm()
            if distance is not None and distance < HARD_MIN_DISTANCE_CM:
                return "obstacle"
        why = self.reflex.must_stop(distance, self.hal.battery())
        if why:
            return why
        if kind == "F" and distance is not None:
            cm, deg = self.hal.odometry()
            left = CELL_CM - (cm - self.motion[1]) if self.motion is not None else CELL_CM
            if distance < left + self.reflex.STOP_DISTANCE_CM:
                return "obstacle"
        return None

    def give_up(self, why):
        """The way stayed blocked: a refusal if the robot never moved (it is where it was), a stop if
        it did (somewhere between two cells: an adult must come)."""
        cm, deg = self.hal.odometry()
        moved = abs(cm - self.motion[1]) > 0.5 if self.motion[0] == "F" else abs(deg - self.motion[1]) > 1
        self.paused_since = None
        if moved:
            self.halt(why)
            return
        self.hal.drive(0, 0)
        self.motion = None
        self.last_stop = why
        self.say("EVT REFUSED " + why)

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
        self.pose += 1
        kind = self.motion[0]
        self.motion = None
        self.say("EVT DONE " + kind)

    def halt(self, why):
        self.hal.drive(0, 0)
        self.last_stop = why
        self.paused_since = None
        if self.motion is not None:
            self.motion = None
            self.pose += 1
            self.wait_adult = True
            self.remember("STOP:" + why)
            self.say("EVT STOP " + why)

    def refusal(self, kind):
        if self.estop:
            return "estop"
        if self.wait_adult:
            return "wait_adult"
        if self.motion is not None:
            return "busy"
        if self.hal.bumper():
            return "bump"
        why = self.stop_reason(kind)
        if why and why != "obstacle":
            return why
        # Something in the way: the move starts by waiting for it to clear (see tick).
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
        elif word == "EVENTS" and len(parts) == 2 and parts[1].isdigit():
            after = int(parts[1])
            text = "OK EVENTS now=%d next=%d" % (self.hal.ticks_ms(), self.next_event)
            for number, ms, name in self.events:
                if number > after:
                    text += " %d:%d:%s" % (number, ms, name)
            self.say(text)
        elif word == "STOP" and len(parts) == 1:
            self.halt("command")
            self.say("OK STOP")
        elif word == "LED" and len(parts) == 2 and parts[1] in COLORS:
            self.light = parts[1]
            if not self.warning:
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
            self.view, self.view_key, self.candidate = [], None, None  # nothing seen here is noted once it moves
            self.say("OK " + word + " " + kind)  # the next tick drives, or waits for the way to clear
        else:
            self.say("ERR unknown")

    def state_line(self):
        distance = self.hal.distance_cm()
        card = self.card_in_front()
        return "OK STATE dist=%s card=%s battery=%d estop=%d wait_adult=%d moving=%d last_stop=%s bumper=%d warning=%d resumes=%d pose=%d" % (
            "none" if distance is None else str(int(distance)),
            card or "none",
            int(self.hal.battery()),
            1 if self.estop else 0,
            1 if self.wait_adult else 0,
            0 if self.motion is None else 1,
            self.last_stop,
            1 if self.bumped else 0,
            1 if self.warning else 0,
            self.resumes,
            self.pose,
        )
