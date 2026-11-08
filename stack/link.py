"""The link to the robot: a serial port, nothing else. Not a tier of the robot's stack: a transport.

The Referee is its only user. It sends one line of text per command and reads one line per answer, in the
protocol of robot/firmware.py; it decides nothing. Today: the USB cable of the robot, or the pseudo-terminal of
the simulator (its path is in .robot-sim/serial); ROBOT_SERIAL chooses another port. BLE is TO BE COMPLETED.
"""
import os
import termios
import tty

import clock


class RobotNotConnected(Exception):
    pass


class Link:
    def __init__(self, state_dir):
        self.state_dir = state_dir
        self.fd = None
        self.path = None
        self.buffer = b""

    def port(self):
        path = os.environ.get("ROBOT_SERIAL")
        if not path:
            try:
                path = (self.state_dir / "serial").read_text(encoding="utf-8").strip()
            except OSError:
                raise RobotNotConnected("no robot: start the simulator, or set ROBOT_SERIAL") from None
        return path

    def connect(self):
        path = self.port()
        # A stack may keep running for hours: when the robot's port changes (the simulator restarted), connect to
        # the new one.
        if self.fd is not None and path == self.path:
            return
        self.close()
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise RobotNotConnected(f"the robot is not connected ({path}: {e.strerror})") from None
        # TCSANOW: keep what the robot already said (an old answer is dropped by drain(), an event is read).
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200  # the real robot; no effect on the simulator
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        self.fd, self.buffer, self.path = fd, b"", path
        self.drain()
        reply = self.request("HELLO")
        if not reply.startswith("OK HELLO robot"):
            self.close()
            raise RobotNotConnected(f"this is not the robot (it answered {reply!r})")

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None

    def send(self, line):
        try:
            os.write(self.fd, (line + "\n").encode("ascii"))
        except OSError as e:
            self.close()
            raise RobotNotConnected(f"the link to the robot is broken ({e.strerror})") from None

    def read_line(self, timeout):
        end = clock.monotonic() + timeout
        while b"\n" not in self.buffer:
            left = max(0.0, end - clock.monotonic())
            ready = clock.readable(self.fd, left)  # with 0: look once, do not wait
            if not ready and left == 0:
                return None
            if ready:
                try:
                    self.buffer += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
                except OSError as e:
                    self.close()
                    raise RobotNotConnected(f"the link to the robot is broken ({e.strerror})") from None
        line, self.buffer = self.buffer.split(b"\n", 1)
        return line.decode("ascii", "replace").strip()

    def drain(self):
        """Read what the robot said while nobody asked (an event, an old answer). Nothing depends on catching an
        event: the robot keeps its last events, read again with EVENTS."""
        while self.read_line(0) is not None:
            pass

    def request(self, line, timeout=1.0):
        """Send a command and return its answer: "OK <the same word> …" or "ERR …". An older answer still on the
        wire (OK PONG, the OK STOP of a move that timed out) is skipped: taken for this answer, it would shift every
        answer after it."""
        self.send(line)
        word = line.split()[0]
        end = clock.monotonic() + timeout
        while True:
            reply = self.read_line(max(0.0, end - clock.monotonic()))
            if reply is None:
                raise RobotNotConnected("the robot does not answer")
            if reply.startswith("EVT "):
                continue
            if reply.startswith("OK ") and not reply.startswith("OK " + word):
                continue
            return reply
