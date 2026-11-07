"""The program of the robot. It runs in the simulator, and is written to run unchanged on the robot.

It is written in the part of Python that MicroPython understands (no import, no f-string, no type
annotation), because the robot's board runs MicroPython. On the real board, a small file gives it
the HAL below; that file is TO BE COMPLETED once the robot is built.

The robot reads one command per line from the computer (the Referee) and answers one line. It
moves four cells at most per command, and always stops on a cell. Its reflexes (reflex.py) can stop
it at any time, without asking the computer. Nothing else is understood.

    HELLO          ->  OK HELLO robot <protocol>
    PING           ->  OK PONG              (the computer says it is still there)
    STATE          ->  OK STATE dist=<cm|none> card=<label|none> battery=<percent>
                       estop=<0|1> wait_adult=<0|1> moving=<0|1> last_stop=<reason> bumper=<0|1>
                       warning=<0|1> resumes=<number of presses on Resume since the robot started>
                       pose=<number of the robot's place: +1 after each cell, stop or Resume>
                       slid=<cells reached in the last move: MOVE F 0 or 1, MOVE <d> <n> 0 to n, a turn 0>
                       looked=<1 when it stands still and the camera's view of where it stands is believed>
                       heading=<degrees from north, from the compass (magnetometer), or none>
                       boxes=<none, or the points of boxes the camera sees, once its view is believed: <angle>:<cm>,...>
                       fault=<none|reflex|error: the robot's program had an error, see FAULT below>
                       tick_max=<the longest time between two turns of the robot's loop since the last STATE, ms>
                       (an adult puts the robot back on the last cell it reached, even when the
                       computer missed an EVT CELL)
                       dist= is "none" when the last reading is too old to be believed (see the sensors below)
    MOVE F         ->  OK MOVE F, then EVT DONE F, EVT STOP <reason> (stopped on the way: an adult must
                       come), or EVT REFUSED <reason> (it waited, the way stayed blocked, it never moved)
    MOVE <d> <n> [UNTIL <conditions>]
                   ->  slide n cells (1 to MAX_CELLS) without turning: d is F (forwards), B (backwards),
                       L (to its left) or R (to its right). OK MOVE <d> <n>, then EVT CELL <cells done>
                       <pose> on each cell it reaches, and at the end EVT DONE <d> <cells done> <why>:
                       "all", "card" (a card right in front of the camera, the way it faces), "ball" (the
                       camera sees a ball), "new" (the camera shows a kind of card, a name, it did not
                       show when the move began), "obstacle" (forwards: the way stayed blocked, it
                       stayed on the last cell), "sensor" (forwards: the distance sensor gave no recent reading,
                       it stayed on the last cell), "camera" (with conditions: no image since it stopped on that
                       cell, it does not go on without looking), or "bump", "estop", "watchdog", "command" (STOP), "fault" when
                       that happens while it stands on a cell between two cells: it is on that cell, no
                       adult needed (after "estop" it still refuses to move until Resume). Conditions,
                       with commas: CARD (forwards, also checked before it starts, so it never drives
                       onto that card), BALL, NEW; with any of them, the robot stands on each cell until
                       the camera's view is believed (CELL_LOOK_MS at least), so that it can look. EVT STOP
                       and EVT REFUSED as for MOVE F; after EVT STOP, the adult puts it back on the last
                       cell it reached.
    TURN L, TURN R ->  OK TURN L, then EVT DONE L, or EVT STOP <reason>
    LED <color>    ->  OK LED <color>
    STOP           ->  OK STOP
    HOLD           ->  OK HOLD: stop on the next cell, as soon as possible. During MOVE <d> <n>, the robot finishes
                       the cell it is driving to (or stops at once if it stands on a cell between two cells), then
                       EVT DONE <d> <cells done> hold: no adult needed. MOVE F and a quarter turn end on a cell
                       anyway; standing still, nothing happens.
    EVENTS <n>     ->  OK EVENTS now=<ms> next=<id> <id>:<ms>:<event> ...  (the remembered events after n)
    anything else  ->  ERR <reason>
and, without being asked:
    EVT BUMP       something touched the bumper (a ball, a foot), moving or not
    EVT NEAR       standing still, something came close in front: the light warns (reflex.warn)
    EVT SEEN:<card>:<angle>:<cm>:<pose>  standing still, the camera recognised this card, this many
                   degrees to the right (negative: to the left) and this far from the robot's centre
    EVT BALL:<trend>:<angle>:<cm>:<angle>:<cm>:<pose>  standing still, the camera sees a ball (two
                   sightings, half a second apart) and how it moves: closer, away, still, or gone
    EVT RESUMED    an adult pressed Resume on the robot
    EVT FAULT:<kind>  the robot's own program had an error: "reflex" (in reflex.py) or "error" (anywhere else). It
                   stops its wheels at once (EVT STOP fault when it was driving), then refuses to move (ERR fault)
                   until an adult presses Resume. A command that met the error is answered ERR fault.
When something is in its way during a forward move, the robot stops its motors and waits up to
reflex.OBSTACLE_WAIT_MS for the way to clear (PAUSE, then GO_ON), then finishes the same move. When the
distance sensor's last reading is too old (DISTANCE_MAX_AGE_MS), it stops its motors the same way and waits
up to SENSOR_WAIT_MS for a new one: then EVT REFUSED sensor, or EVT STOP sensor if it had moved.
When two turns of its loop are further apart than LATE_TICK_MS while its wheels turn (the program was
late: nothing watched the way), it stops: EVT STOP late, an adult must come.

The robot also remembers its last 32 events (those, ESTOP, STOP:<reason> when a move stops, PAUSE and
GO_ON, FAULT:<kind>, and
GONE:<card>:<angle>:<cm>:<pose> when a card it saw leaves its view while it stands still), numbered,
with the time of its clock: a computer that was not listening asks EVENTS. The camera is only
believed while the robot stands still, and only from images taken after it stopped (the camera answers
late): during a move or a turn, nothing it sees is noted.

Its wheels (mecanum wheels) slide it forwards, backwards or to a side, and turn it on the spot. Its
distance sensor and its camera only look forwards: backwards or sideways, nothing tells it what is in
the way, so it goes there slowly (HARD_BLIND_SPEED_CM_S), and the Referee sends it there only into
cells its sensor has seen free. Its bumper goes all around it: when something touches it, a move stops
at once, whatever reflex.py says.

After a move is interrupted while it drives (obstacle, watchdog, emergency button, STOP), the robot is
somewhere between two cells (not when it stood on a cell between two cells of a slide: EVT DONE above). It refuses to move until an adult puts it back on the cell where the move
started (for a move of several cells: the last cell it reached) and presses the Resume button on the
robot (EVT RESUMED, and "resumes" in STATE goes up by one). Nothing from the computer can do that.

What the program needs from the board (hal). Three kinds of inputs: what is brief (a touch, a press) is
counted by the board as it happens, so that a touch shorter than a turn of the loop is never missed; what is
counted (the wheels) is a total; what is measured comes with the time of the measure, and is never waited for.
    ticks_ms(), ticks_diff(new, old)   a clock in milliseconds
    read_lines(), write_line(text)     the link to the computer, never waited for
    drive(forward_cm_s, right_cm_s, turn_deg_s)
                                       the four wheels: speed forwards (- = backwards), speed to the
                                       right (- = to the left), turning speed (+ = right). Ignored (the
                                       wheels stay stopped) while the motors are cut, see bumps()
    release(bumps)                     the program has dealt with the touches up to bumps (and the press):
                                       drive() works again, unless the bumper was touched again since, or
                                       the stop button is still pressed. The program sends drive(0, 0, 0)
                                       right after, so that the motors never take back an older order
    odometry()                         (centimetres travelled, degrees turned) counted by the wheels
                                       since start; how true sideways moves are on the real wheels is
                                       TO BE COMPLETED (mecanum wheels slip more sideways)
    distance_cm()                      the last reading of the distance sensor in front: (a number or
                                       None, ticks_ms() when it was measured)
    battery()                          percent
    button(name)                       "stop" or "resume": True while pressed
    presses(name)                      how many times that button was pressed since start
    bumper()                           True while something touches the bumper
    bumps()                            how many times something touched the bumper since start. On a touch,
                                       or a press on "stop", the board cuts the motors itself, at once,
                                       until release()
    camera_view()                      the camera's newest answer: (number of the image, ticks_ms() when
                                       the image was taken, what it recognises: a list of [name ("ball",
                                       "box" for a point of a box's side it sees, or a card), angle in
                                       degrees (+ = right), distance in cm])
    compass()                          the magnetometer: degrees from north (0 to 360); its calibration on
                                       the real board (motors and magnets nearby) is TO BE COMPLETED
    led(color)                         one of COLORS

On the real board, a small file (TO BE COMPLETED once the robot is built) gives this hal: interrupts that only
count (bumper, buttons, wheels) and cut the motors' enable pins on a touch, the distance sensor read without
waiting, the camera's bytes kept by the serial port. It calls tick() every TICK_MS and feeds the board's own
watchdog (machine.WDT) after each tick: a program stuck in a tick restarts the board, motors off (how the motor
driver behaves during that restart is TO BE COMPLETED).
"""

PROTOCOL = 5

# The length of one cell of the mat (centimetres): the same number as "cell_cm" in mat.json.
CELL_CM = 30

# Limits that reflex.py cannot loosen.
HARD_MAX_SPEED_CM_S = 20
HARD_MIN_DISTANCE_CM = 3
# Backwards or sideways, where the distance sensor does not look: slower, so that a touch on the
# bumper stops it sooner. A choice of the simulator; on the real robot TO BE COMPLETED.
HARD_BLIND_SPEED_CM_S = 5
TURN_DEG_S = 90
# The board calls tick() every TICK_MS. While the wheels turn, two ticks further apart than LATE_TICK_MS
# mean that nothing watched the way for that long: the robot stops. On the real board TO BE COMPLETED
# (measure how long a tick takes, garbage collection included).
TICK_MS = 10
LATE_TICK_MS = 50
# Forwards, a distance measured longer ago than this is not believed (reflex.py may ask for less): the
# robot stops its motors and waits up to SENSOR_WAIT_MS for a new one. The simulator's sensor measures every
# 30 ms; the real one's rate is TO BE COMPLETED (20 to 100 ms for the VL53L1X).
HARD_DISTANCE_MAX_AGE_MS = 300
SENSOR_WAIT_MS = 1000
FAULTS = ("reflex", "error")
SLIDES = {"F": (1, 0), "B": (-1, 0), "L": (0, -1), "R": (0, 1)}  # (forwards, to the right)
CONDITIONS = ("CARD", "BALL", "NEW")
MAX_CELLS = 4  # the most cells one command may move

COLORS = ("off", "red", "orange", "green", "blue")
EVENTS_KEPT = 32
# The camera's answer must stay the same this long before the robot believes it (milliseconds of images,
# all taken after the robot stopped). The simulator's value; on the real robot, the camera answers 1 to 2 s
# late (RobotIA): TO BE COMPLETED.
CAMERA_STABLE_MS = 200
# A move with conditions: on each cell, the robot stands at least this long, and until the camera's
# view is believed, but never longer than CELL_LOOK_MAX_MS (a camera that never settles: it decides with what
# its newest image shows; no image at all since it stopped: EVT DONE <d> <k> camera). CELL_LOOK_MAX_MS must be
# longer than the camera's delay plus CAMERA_STABLE_MS.
CELL_LOOK_MS = CAMERA_STABLE_MS + 100
CELL_LOOK_MAX_MS = 2000
# How often the robot looks again at a moving ball to tell whether it comes closer (milliseconds).
BALL_SAMPLE_MS = 500
BALL_MOVE_CM = 2  # a change of distance smaller than this: the ball does not move


class ReflexError(Exception):
    """An error inside reflex.py (the file the participants change)."""


class Firmware:
    def __init__(self, hal, reflex):
        self.hal = hal
        self.reflex = reflex
        # None, or [kind, start]: kind "F", "B", "L" or "R" for one cell of a slide ("F" also for MOVE F),
        # "TL" or "TR" for a quarter turn, "P" while it stands on a cell between two cells of a slide
        self.motion = None
        # A move of several cells (MOVE <d> <n>): [direction, cells asked, cells done, conditions,
        # card names the camera showed since the move began], or None
        self.plan = None
        self.slid = 0  # cells reached in the last MOVE <d> <n> (STATE slid=)
        self.hold = False  # HOLD received: the slide stops on the next cell
        self.paused_since = None  # while it waits for the way to clear
        self.estop = False
        self.wait_adult = False
        self.last_stop = "none"
        self.bumped = False
        self.resumes = 0
        self.light = "off"  # the colour asked by the computer
        self.warning = False
        self.events = []  # [number, time in ms, name], the last EVENTS_KEPT
        self.next_event = 1
        self.pose = 0  # +1 each time the robot may be somewhere else: a move ends, stops, or Resume
        self.ball = None  # the last sighting of a ball [angle, cm, time], while standing still
        self.ball_trend = None
        self.view = []  # what the camera shows, once believed: [[card, angle, cm], ...]
        self.view_key = None
        self.boxes = []  # the points of boxes in the believed view: [[angle, cm], ...]
        self.candidate = None
        self.candidate_since = 0
        self.last_heard = hal.ticks_ms()
        self.fault = None  # None, or one of FAULTS until an adult presses Resume
        self.fault_text = None  # what went wrong, for the adult (the simulator's page shows it)
        self.last_tick = self.last_heard
        self.tick_max = 0  # the longest gap between two ticks since the last STATE (ms)
        self.bump_count = hal.bumps()
        self.press_count = {"stop": hal.presses("stop"), "resume": hal.presses("resume")}
        self.driving = False  # the wheels were last told to turn
        self.still_since = self.last_heard  # when they were last told to stop
        self.paused_for = None  # "obstacle" or "sensor", while it waits with its motors stopped
        self.resume_pending = False  # Resume pressed while the stop button was still held: counted when it is let go
        # What the sensors gave in this tick: read once, at its start.
        self.dist, self.dist_at = None, self.last_heard
        self.battery = 100
        self.frame, self.frame_at, self.seen = None, None, []
        self.last_frame = None  # the last image looked at
        # A touch the board counted before the program started: dealt with, the wheels stay stopped.
        hal.release(self.bump_count)
        hal.drive(0, 0, 0)

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
        """Called every TICK_MS by the board (every 10 ms in the simulator). An error anywhere in it stops the
        wheels: the robot never drives on with nobody watching the way."""
        hal = self.hal
        now = hal.ticks_ms()
        gap = hal.ticks_diff(now, self.last_tick)
        was_driving = self.driving  # the wheels turned during the gap
        self.last_tick = now
        if gap > self.tick_max:
            self.tick_max = gap
        try:
            self.sample()
        except Exception as e:
            # A sensor that fails: the last readings get old (not believed), the robot still hears its commands
            # and its buttons, so that an adult can release it.
            self.failed("error", "sensor: " + repr(e))
        try:
            self.inputs()  # before the commands: a touch counted now refuses a move asked now
            for line in hal.read_lines():
                self.last_heard = now
                self.answer_safely(line.strip())
            self.run(now, gap, was_driving)
        except ReflexError as e:
            self.failed("reflex", str(e))
        except Exception as e:
            self.failed("error", repr(e))

    def sample(self):
        """Read the measures once per tick, never waiting: every decision of the tick uses these."""
        hal = self.hal
        self.dist, self.dist_at = hal.distance_cm()
        self.battery = hal.battery()
        self.frame, self.frame_at, self.seen = hal.camera_view()

    def answer_safely(self, line):
        try:
            self.answer(line)
        except ReflexError as e:
            self.say("ERR fault")
            self.failed("reflex", str(e))
        except Exception as e:
            self.say("ERR fault")
            self.failed("error", repr(e))

    def failed(self, kind, text):
        """An error in the program: the wheels first, then the robot's state, then the words. It refuses to move
        until Resume."""
        self.hal.drive(0, 0, 0)
        first = self.fault is None
        if first:
            self.fault, self.fault_text = kind, text[:160]
        try:
            self.halt("fault")
        except Exception:
            self.motion, self.plan = None, None  # whatever failed while saying it, nothing moves any more
        if first:
            try:
                self.note("FAULT:" + kind)
            except Exception:
                pass

    def ask(self, name, *args):
        """Call a function of reflex.py: an error there is the participants' file, not the firmware."""
        try:
            return getattr(self.reflex, name)(*args)
        except Exception as e:
            raise ReflexError(name + ": " + repr(e))

    def const(self, name, default=None):
        """A number of reflex.py: anything else there is an error in the participants' file."""
        value = getattr(self.reflex, name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ReflexError(name + " is not a number")
        return value

    def counted(self, name):
        """True when the button was pressed since the last tick (counted by the board, even a short press)."""
        n = self.hal.presses(name)
        pressed = n != self.press_count[name]
        self.press_count[name] = n
        return pressed

    def drive(self, forward, right, turn):
        self.hal.drive(forward, right, turn)
        moving = forward != 0 or right != 0 or turn != 0
        if self.driving and not moving:
            self.still_since = self.hal.ticks_ms()
        self.driving = moving

    def inputs(self):
        """The buttons and the bumper, counted by the board since the last tick."""
        hal = self.hal
        stopped = self.counted("stop")
        if (stopped or hal.button("stop")) and not self.estop:
            self.estop = True
            self.remember("ESTOP")
            self.halt("estop")
        bumps = hal.bumps()
        if bumps != self.bump_count:
            # Counted by the board, which already cut the motors: even a touch shorter than a tick is here.
            self.bump_count = bumps
            self.note("BUMP")
            self.halt("bump")
            if not self.estop:
                hal.release(bumps)  # dealt with: the wheels obey drive() again, unless touched again since
                hal.drive(0, 0, 0)  # and their last order is "stop"
        self.bumped = hal.bumper()
        if self.counted("resume") and not stopped:  # a stop pressed in the same tick wins
            self.resume_pending = True
        if self.resume_pending and hal.button("stop") and not hal.button("resume"):
            self.resume_pending = False  # pressed and let go while the stop button was held: nothing
        if self.resume_pending and not hal.button("stop"):
            # Counted at each press, even when the robot was not waiting: the computer compares the
            # number, it does not need to catch the message.
            self.resume_pending = False
            self.estop = False
            self.wait_adult = False
            self.fault, self.fault_text = None, None
            self.resumes += 1
            self.pose += 1  # the adult put it back: a new place, where nothing seen before counts
            self.view, self.view_key, self.candidate, self.last_frame = [], None, None, None
            self.ball, self.ball_trend = None, None
            self.still_since = hal.ticks_ms()
            hal.release(self.bump_count)
            hal.drive(0, 0, 0)
            self.note("RESUMED")

    def run(self, now, gap, was_driving):
        hal = self.hal
        if self.motion is None:
            if self.fault is None:
                self.warn()
            self.look(now)
            return
        kind = self.motion[0]
        if self.fault is not None:
            self.halt("fault")  # never steer while the program is in error
            return
        if was_driving and gap > LATE_TICK_MS:
            self.halt("late")
            return
        if hal.ticks_diff(now, self.last_heard) > self.const("WATCHDOG_MS"):
            self.halt("watchdog")
            return
        if kind == "P":
            # Standing on a cell of a slide (or on its first cell, to look before it starts): the camera
            # looks, then the conditions decide.
            self.drive(0, 0, 0)
            self.look(now)
            stood = hal.ticks_diff(now, self.motion[1])
            if (stood >= CELL_LOOK_MS and self.view_key is not None) or stood >= CELL_LOOK_MAX_MS:
                if self.view_key is None and self.fresh_view() is None:
                    # No image taken since it stopped: it cannot tell, so it does not go on blind.
                    self.end_plan("camera")
                    return
                why = self.first_look() if self.plan[2] == 0 else self.condition_met()
                if why:
                    self.end_plan(why)
                else:
                    self.start_cell()
            return
        why = self.stop_reason(kind)
        if why in ("obstacle", "sensor") and not hal.bumper():
            # Something in the way, not touching (a ball rolling past), or no recent distance: the motors stop,
            # it waits for the way to clear or for a new reading.
            self.drive(0, 0, 0)
            if self.paused_since is None:
                self.paused_since = now
                if why == "obstacle":
                    self.remember("PAUSE")
            self.paused_for = why
            wait = self.const("OBSTACLE_WAIT_MS", 0) if why == "obstacle" else SENSOR_WAIT_MS
            if hal.ticks_diff(now, self.paused_since) > wait:
                self.give_up(why)
            return
        if why:
            self.halt(why)
            return
        if self.paused_since is not None:
            if self.paused_for == "obstacle":
                self.remember("GO_ON")
            self.paused_since, self.paused_for = None, None
        self.steer()

    def warn(self):
        """Standing still: the light warns when something is close. Only the light."""
        color = None
        if hasattr(self.reflex, "warn"):
            distance = self.distance()
            color = self.ask("warn", None if distance == "old" else distance)
        if color in COLORS:
            if not self.warning:
                self.warning = True
                self.note("NEAR")
            self.hal.led(color)
        elif self.warning:
            self.warning = False
            self.hal.led(self.light)

    def distance(self):
        """The distance in front (a number or None), or "old" when the last reading is too old to be believed."""
        try:
            limit = min(self.const("DISTANCE_MAX_AGE_MS", HARD_DISTANCE_MAX_AGE_MS), HARD_DISTANCE_MAX_AGE_MS)
        except ReflexError:
            limit = HARD_DISTANCE_MAX_AGE_MS  # STATE still answers; must_stop will say what is wrong
        if self.hal.ticks_diff(self.hal.ticks_ms(), self.dist_at) > limit:
            return "old"
        return self.dist

    def fresh_view(self):
        """What the camera's newest image shows, if it was taken after the robot stopped; None otherwise (it is
        driving, or the image shows where it was before)."""
        if self.driving or self.frame_at is None or self.hal.ticks_diff(self.frame_at, self.still_since) < 0:
            return None
        return self.seen

    def look(self, now):
        """Standing still: once the camera's view is stable, note each card that came into it, and
        each card that left it (moved by hand, hidden by a box). Only images taken after it stopped, each once."""
        everything = self.fresh_view()
        if everything is None or self.frame == self.last_frame:
            return
        self.last_frame = self.frame
        taken = self.frame_at
        self.watch_ball(taken, [v for v in everything if v[0] == "ball"])
        view = [v for v in everything if v[0] not in ("ball", "box")]  # the cards: they stay put, the ball does not
        boxes = [[v[1], v[2]] for v in everything if v[0] == "box"]
        key = repr(view) + repr(boxes)
        if key != self.candidate:
            self.candidate, self.candidate_since = key, taken
            return
        if key == self.view_key or self.hal.ticks_diff(taken, self.candidate_since) < CAMERA_STABLE_MS:
            return
        left = list(self.view)
        self.view, self.view_key, self.boxes = view, key, boxes
        for card, angle, cm in view:
            if [card, angle, cm] in left:
                left.remove([card, angle, cm])
            else:
                self.note("SEEN:%s:%d:%d:%d" % (card, angle, cm, self.pose))
        for card, angle, cm in left:
            self.remember("GONE:%s:%d:%d:%d" % (card, angle, cm, self.pose))

    def watch_ball(self, taken, balls):
        """Twice a second of images, compare the nearest ball with the last sighting: note when it starts to come
        closer, to go away, to stand still, or leaves the camera's view."""
        if self.ball is not None and self.hal.ticks_diff(taken, self.ball[2]) < BALL_SAMPLE_MS:
            return
        if not balls:
            if self.ball_trend is not None:
                angle, cm = self.ball[0], self.ball[1]
                self.note("BALL:gone:%d:%d:%d:%d:%d" % (angle, cm, angle, cm, self.pose))
            self.ball, self.ball_trend = None, None
            return
        nearest = balls[0]
        for b in balls:
            if b[2] < nearest[2]:
                nearest = b
        if self.ball is not None:
            change = nearest[2] - self.ball[1]
            trend = "closer" if change < -BALL_MOVE_CM else "away" if change > BALL_MOVE_CM else "still"
            if trend != self.ball_trend:
                self.note("BALL:%s:%d:%d:%d:%d:%d" % (trend, self.ball[0], self.ball[1], nearest[1], nearest[2], self.pose))
                self.ball_trend = trend
        self.ball = [nearest[1], nearest[2], taken]

    def shown(self):
        """What the camera shows of where the robot stands: its believed view, else its newest image taken
        after it stopped, else nothing."""
        if self.view_key is not None:
            return self.view
        return self.fresh_view() or []

    def card_in_front(self):
        """The nearest card just in front (the cell ahead), from what the camera shows."""
        best = None
        for card, angle, cm in self.shown():
            # 40 cm: a card on the next cell is seen at about 30 cm, one two cells away at about 60 cm; with the
            # camera's error, 45 cm took a card two cells away for one in front (review of 2026-11-06).
            if card not in ("ball", "box") and -15 <= angle <= 15 and cm <= CELL_CM * 4 // 3 and (best is None or cm < best[1]):
                best = [card, cm]
        return None if best is None else best[0]

    def first_look(self):
        """On the cell where a slide starts, once the camera has looked: the names it shows, and "card" when it
        would drive forwards onto one."""
        names = self.plan[4]
        for card, angle, cm in self.shown():
            if card not in ("ball", "box") and card not in names:
                names.append(card)
        if self.plan[0] == "F" and "CARD" in self.plan[3] and self.card_in_front():
            return "card"
        return None

    def condition_met(self):
        """On a cell of a slide, after looking: the first condition of the move that is true, or None."""
        names = self.plan[4]
        why = None
        for condition in self.plan[3]:
            if condition == "CARD" and self.card_in_front():
                why = why or "card"
            if condition == "BALL":
                for seen in self.fresh_view() or []:
                    if seen[0] == "ball":
                        why = why or "ball"
            if condition == "NEW":
                for card, angle, cm in self.view:
                    if card not in names:
                        why = why or "new"
        for card, angle, cm in self.view:
            if card not in names:
                names.append(card)
        return why

    def stop_reason(self, kind):
        """Why the robot cannot go on now, or None. "obstacle": something in the way of the rest of the
        cell (closer than what is left to go, plus the stopping distance). "sensor": forwards, the distance
        sensor's last reading is too old. Only forwards: the distance sensor sees nothing backwards or
        sideways (nor while it turns)."""
        distance = None
        if kind == "F":
            distance = self.distance()
            if distance == "old":
                return "sensor"
            if distance is not None and distance < HARD_MIN_DISTANCE_CM:
                return "obstacle"
        why = self.ask("must_stop", distance, self.battery)
        if why:
            return why
        if kind == "F" and distance is not None:
            cm, deg = self.hal.odometry()
            left = CELL_CM - (cm - self.motion[1]) if self.motion is not None and self.motion[0] == "F" else CELL_CM
            if distance < left + self.const("STOP_DISTANCE_CM"):
                return "obstacle"
        return None

    def give_up(self, why):
        """The way stayed blocked: a refusal if the robot never moved (it is where it was), a stop if
        it did (somewhere between two cells: an adult must come)."""
        cm, deg = self.hal.odometry()
        moved = abs(cm - self.motion[1]) > 0.5 if self.motion[0] in SLIDES else abs(deg - self.motion[1]) > 1
        self.paused_since, self.paused_for = None, None
        if moved:
            self.halt(why)
            return
        self.drive(0, 0, 0)
        if self.plan is not None and self.plan[2] > 0:
            # It is on the last cell it reached: a move that ends early, not a refusal.
            self.last_stop = why
            self.end_plan(why)
            return
        self.motion = None
        self.plan = None
        self.last_stop = why
        self.say("EVT REFUSED " + why)

    def steer(self):
        """Drive towards the end of the cell (or of the quarter turn), slower near the end, and stop there."""
        cm, deg = self.hal.odometry()
        kind, start = self.motion
        if kind in SLIDES:
            left = CELL_CM - (cm - start)
            if left <= 0:
                self.finish()
                return
            top = min(self.const("MAX_SPEED_CM_S"), HARD_MAX_SPEED_CM_S)
            if kind != "F":
                top = min(top, HARD_BLIND_SPEED_CM_S)
            speed = max(2, min(top, left * 3))
            forward, right = SLIDES[kind]
            self.drive(speed * forward, speed * right, 0)
        else:
            left = 90 - abs(deg - start)
            if left <= 0:
                self.finish()
                return
            rate = max(10, min(TURN_DEG_S, left * 4))
            if kind == "TL":
                rate = -rate
            self.drive(0, 0, rate)

    def finish(self):
        """The end of a cell, or of a quarter turn."""
        self.drive(0, 0, 0)
        self.pose += 1
        kind = self.motion[0]
        if self.plan is None:
            self.motion = None
            if kind == "F":
                self.slid = 1  # MOVE F reached its cell
            self.say("EVT DONE " + kind[-1])
            return
        plan = self.plan
        plan[2] += 1
        self.slid = plan[2]
        self.say("EVT CELL %d %d" % (plan[2], self.pose))
        if plan[2] >= plan[1]:
            self.end_plan("all")
        elif self.hold:
            self.end_plan("hold")
        elif plan[3]:
            self.motion = ["P", self.hal.ticks_ms()]
            self.view, self.view_key, self.candidate = [], None, None
        else:
            self.start_cell()

    def start_cell(self):
        """The next cell of a slide: its odometry starts here. A new list each time: the simulator
        notes where each cell starts (where an adult puts the robot back)."""
        cm, deg = self.hal.odometry()
        self.motion = [self.plan[0], cm]
        self.view, self.view_key, self.candidate = [], None, None  # nothing seen is noted while it moves
        self.ball, self.ball_trend = None, None

    def end_plan(self, why):
        self.drive(0, 0, 0)
        self.hold = False
        plan = self.plan
        self.motion = None
        self.plan = None
        self.say("EVT DONE %s %d %s" % (plan[0], plan[2], why))

    def halt(self, why):
        self.drive(0, 0, 0)
        self.hold = False
        self.last_stop = why
        self.paused_since, self.paused_for = None, None
        if self.motion is not None and self.motion[0] == "P":
            # Standing on a cell between two cells of a slide: it is on that cell, no adult needed.
            self.end_plan(why)
            return
        self.plan = None
        if self.motion is not None:
            self.motion = None
            self.pose += 1
            self.wait_adult = True
            self.remember("STOP:" + why)
            self.say("EVT STOP " + why)

    def refusal(self, kind):
        if self.fault is not None:
            return "fault"
        if self.estop:
            return "estop"
        if self.wait_adult:
            return "wait_adult"
        if self.motion is not None:
            return "busy"
        if self.hal.bumper() or self.hal.bumps() != self.bump_count:
            return "bump"
        why = self.stop_reason(kind)
        if why and why not in ("obstacle", "sensor"):
            return why
        # Something in the way, or no recent distance: the move starts by waiting (see run).
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
        elif word == "HOLD" and len(parts) == 1:
            self.say("OK HOLD")
            if self.plan is not None and self.motion is not None and self.motion[0] == "P":
                self.end_plan("hold")  # it stands on a cell between two cells: it stays there
            elif self.plan is not None:
                self.hold = True
        elif word == "LED" and len(parts) == 2 and parts[1] in COLORS:
            self.light = parts[1]
            if not self.warning:
                self.hal.led(parts[1])
            self.say("OK LED " + parts[1])
        elif (word == "MOVE" and parts[1:] == ["F"]) or (word == "TURN" and parts[1:] in (["L"], ["R"])):
            kind = parts[1] if word == "MOVE" else "T" + parts[1]
            why = self.refusal(kind)
            if why:
                self.say("ERR " + why)
                return
            cm, deg = self.hal.odometry()
            self.slid = 0
            self.motion = [kind, cm if kind == "F" else deg]
            self.view, self.view_key, self.candidate = [], None, None  # nothing seen here is noted once it moves
            self.ball, self.ball_trend = None, None
            self.say("OK " + word + " " + parts[1])  # the next tick drives, or waits for the way to clear
        elif word == "MOVE" and len(parts) in (3, 5) and parts[1] in SLIDES and parts[2].isdigit():
            self.slide(parts)
        else:
            self.say("ERR unknown")

    def slide(self, parts):
        """MOVE <d> <n> [UNTIL <conditions>]: several cells in one direction, without turning."""
        cells = int(parts[2])
        conditions = []
        if len(parts) == 5:
            if parts[3] != "UNTIL":
                self.say("ERR unknown")
                return
            conditions = parts[4].split(",")
        bad = False
        for condition in conditions:
            if condition not in CONDITIONS:
                bad = True
        if cells < 1 or cells > MAX_CELLS or bad:
            self.say("ERR unknown")
            return
        kind = parts[1]
        why = self.refusal(kind)
        if why:
            self.say("ERR " + why)
            return
        self.slid = 0
        self.hold = False
        self.plan = [kind, cells, 0, conditions, []]
        self.say("OK MOVE %s %d" % (kind, cells))
        if conditions and self.view_key is None:
            # Nothing believed of this cell yet (right after a turn, or the camera is late): it stands and looks
            # first, then first_look() decides, as on a cell between two cells.
            self.motion = ["P", self.hal.ticks_ms()]
            self.view, self.view_key, self.candidate = [], None, None
            return
        why = self.first_look()  # the names it shows now; already in front of a card: it does not drive onto it
        if why:
            self.end_plan(why)
            return
        self.start_cell()

    def state_line(self):
        distance = self.distance()
        if distance == "old":
            distance = None
        card = self.card_in_front()
        heading = "none"
        if hasattr(self.hal, "compass"):
            heading = str(int(self.hal.compass()) % 360)
        tick_max, self.tick_max = self.tick_max, 0
        looked = self.motion is None and self.view_key is not None
        boxes = "none"
        if looked and self.boxes:
            boxes = ",".join(["%d:%d" % (angle, cm) for angle, cm in self.boxes])
        return "OK STATE dist=%s card=%s battery=%d estop=%d wait_adult=%d moving=%d last_stop=%s bumper=%d warning=%d resumes=%d pose=%d slid=%d looked=%d heading=%s fault=%s tick_max=%d boxes=%s" % (
            "none" if distance is None else str(int(distance)),
            card or "none",
            int(self.battery),
            1 if self.estop else 0,
            1 if self.wait_adult else 0,
            0 if self.motion is None else 1,
            self.last_stop,
            1 if self.bumped else 0,
            1 if self.warning else 0,
            self.resumes,
            self.pose,
            self.slid,
            1 if looked else 0,
            heading,
            self.fault or "none",
            tick_max,
            boxes,
        )
