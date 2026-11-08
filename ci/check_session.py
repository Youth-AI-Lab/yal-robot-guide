"""The whole robot in this Codespace (bin/robot-session), checked without a model, without a cost, with nothing sent
out: Open WebUI talks to a fake model in this Codespace, the Strategist has no model (each step goes straight to the
Pilot's goal tools).

    python3 -I ci/check_session.py      in this guide's Codespace or dev container (Docker, workshop-key)

What it checks:
  - robot-session start brings up the simulator, the Interpreter's link and Open WebUI, with "Feurisson robot";
  - the model sees Feurisson's instructions and exactly the link's six tools, receives the saved key, and is asked
    to route away from companies that may collect data; the plain model is hidden, and routed the same way;
  - a mission asked through Open WebUI reaches the robot and ends with its report;
  - the filter (interpreter/feurisson_filter.py): the group's language, named by the small model, and the mission as
    it is now go into the instructions; a stop right after a pause in the same answer is refused;
  - the link answers neither on 127.0.0.1 (the Ports tab) nor without its token;
  - a second start is refused and leaves the session running;
  - robot-session stop removes Open WebUI, its conversations, its network and the processes;
  - the key is in no file of the session and on no command line;
  - after a Codespace restart: a pid of pids.json that is now another program is neither counted nor stopped, and
    what an earlier session left in Docker does not stop the next start.
Not checked here: the public ports (only in a real Codespace).

With no key saved, it saves a fake one (sk-or-…) and removes it at the end. With an OpenRouter key saved, that key
goes only to the fake model, here. With another key (sk-ant-…, the kit's default), it stops: robot-session needs an
OpenRouter key, and this check never replaces a key of yours.
"""
import json
import os
import re
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

GUIDE = Path(__file__).resolve().parent.parent
SESSION_CMD = str(GUIDE / "bin" / "robot-session")
PIDS = GUIDE / ".robot-sim" / "session" / "pids.json"
SERVER, BOT, MODEL, FAKE_PORT = "robot-mission", "feurisson-robot", "fake-model", 18116
OWUI, VOLUME, NETWORK = "robot-owui", "robot-owui-data", "robot-session"
OWUI_IMAGE = os.environ.get("ROBOT_OPENWEBUI_IMAGE", "ghcr.io/open-webui/open-webui:v0.12.5")
TOOLS = {f"{SERVER}_{n}" for n in ("mission_options", "start_mission", "pause_mission", "resume_mission",
                                   "stop_mission", "mission_status")}
DENY = {"data_collection": "deny"}
RESULTS = []
CALLS = []
NAMED = []
FRENCH = re.compile(r"\b(le|la|il|est|où|bonjour)\b", re.I)
FILTER = "feurisson_robot"


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(("PASS " if ok else "FAIL ") + name + (f"  ({detail})" if detail and not ok else ""), flush=True)


class FakeModel(BaseHTTPRequestHandler):
    """An OpenAI-compatible model: it calls the tool written "@call <name> <JSON>" in the last message, and notes what
    it receives."""

    def log_message(self, *args):
        pass

    def reply(self, status, payload):
        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.reply(200, {"object": "list", "data": [{"id": MODEL, "object": "model", "owned_by": "fake"}]})

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        messages = body.get("messages") or []
        last = messages[-1] if messages else {}
        text = last.get("content") if isinstance(last.get("content"), str) else ""
        if not body.get("stream"):  # the filter's small model, asked to name the group's language
            NAMED.append({"model": body.get("model"), "text": text, "provider": body.get("provider")})
            named = "French" if FRENCH.search(text) else "English"
            self.reply(200, {"id": "fake", "object": "chat.completion", "model": body.get("model"),
                             "choices": [{"index": 0, "message": {"role": "assistant", "content": named},
                                          "finish_reason": "stop"}]})
            return
        CALLS.append({"authorization": self.headers.get("Authorization", ""), "provider": body.get("provider"),
                      "system": next((m.get("content") for m in messages if m.get("role") == "system"), "") or "",
                      "tools": [(t.get("function") or {}).get("name") for t in body.get("tools") or []],
                      "tool_results": [m.get("content") for m in messages if m.get("role") == "tool"]})
        calls = []
        if last.get("role") != "tool" and text.startswith("@call "):  # one or more: "@call a {} @call b {}"
            for k, piece in enumerate(text.split("@call ")[1:]):
                name, _, args = piece.strip().partition(" ")
                calls.append({"index": k, "id": f"call_{len(CALLS)}_{k}", "type": "function",
                              "function": {"name": name, "arguments": args.strip() or "{}"}})
        base = {"id": f"fake-{len(CALLS)}", "created": int(time.time()), "model": MODEL,
                "object": "chat.completion.chunk"}
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.end_headers()
        delta = {"tool_calls": calls} if calls else {"content": f"FAKE ANSWER ({len(CALLS)})"}
        for d, finish in ((delta, None), ({}, "tool_calls" if calls else "stop")):
            chunk = dict(base, choices=[{"index": 0, "delta": d, "finish_reason": finish}])
            self.wfile.write(f"data: {json.dumps(chunk)}\n\n".encode())
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


def api(method, path, body=None, token=None):
    url = "http://127.0.0.1:3000" + (path if path.startswith("/api/") else "/api/v1" + path)
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    data = None if body is None else json.dumps(body).encode()
    with urllib.request.urlopen(urllib.request.Request(url, data=data, method=method, headers=headers),
                                timeout=60) as r:
        text = r.read().decode()
    return json.loads(text) if text else None


def ask(token, text, model=BOT, limit=90):
    """One message in a new saved conversation, as the browser sends it; what the fake model received meanwhile."""
    n0 = len(CALLS)
    user_id, assistant_id = str(uuid.uuid4()), str(uuid.uuid4())
    known = {c["id"] for c in api("GET", "/chats/?page=1", token=token)}
    api("POST", "/api/chat/completions", {
        "model": model, "stream": True, "messages": [{"role": "user", "content": text}], "parent_id": None,
        "id": assistant_id, "tool_ids": [f"server:mcp:{SERVER}"] if model == BOT else [],
        "user_message": {"id": user_id, "parentId": None, "childrenIds": [], "role": "user", "content": text,
                         "timestamp": int(time.time()), "models": [model]}}, token=token)
    t0, chat_id = time.monotonic(), None
    while time.monotonic() - t0 < limit:
        chat_id = chat_id or next((c["id"] for c in api("GET", "/chats/?page=1", token=token)
                                   if c["id"] not in known), None)
        if chat_id:
            message = api("GET", f"/chats/{chat_id}", token=token)["chat"]["history"]["messages"].get(assistant_id, {})
            if message.get("done"):
                break
        time.sleep(0.5)
    return CALLS[n0:]


def session(*args):
    return subprocess.run([SESSION_CMD, *args], capture_output=True, text=True)


def status():
    return json.loads(session("status").stdout)


def provider():
    helper = subprocess.run(["bash", "-c", "command -v workshop-key"], capture_output=True, text=True).stdout.strip()
    if not helper:
        sys.exit("check_session: run it in this guide's Codespace or dev container (workshop-key).")
    sys.path.insert(0, str(Path(helper).resolve().parent))
    import workshop_provider
    return workshop_provider


def reach(url):
    try:
        urllib.request.urlopen(url, timeout=3)
        return "answered"
    except urllib.error.HTTPError as e:
        return str(e.code)
    except (urllib.error.URLError, OSError):
        return "refused"


def first_session(key, gateway, env):
    t0 = time.monotonic()
    started = subprocess.run([SESSION_CMD, "start"], env=env, capture_output=True, text=True)
    check("robot-session start", started.returncode == 0 and "Ready" in started.stdout,
          (started.stdout + started.stderr)[-600:])
    if started.returncode != 0:
        return
    print(f"     started in {round(time.monotonic() - t0)} s", flush=True)
    now = status()
    check("status: simulator, link and Open WebUI run",
          now["simulator"] and now["link"] and now["open_webui"].startswith("Up"), now)
    check("the link at 127.0.0.1:8790 (the Ports tab): refused", reach("http://127.0.0.1:8790/mcp") == "refused")
    said = reach(f"http://{gateway}:8790/mcp")
    check("the link on Docker's network, without its token: 401", said == "401", said)

    token = api("POST", "/auths/signin", {"email": "", "password": ""})["token"]
    models = {m["id"]: m for m in api("GET", "/api/models", token=token)["data"]}
    check("the chatbot Feurisson robot is there", BOT in models, sorted(models))
    plain = api("GET", f"/models/model?id={MODEL}", token=token) or {}
    check("the plain model is hidden, without built-in tools",
          (plain.get("meta") or {}).get("hidden") is True
          and (plain.get("meta") or {}).get("capabilities", {}).get("builtin_tools") is False, plain.get("meta"))
    seen = ask(token, f"@call {SERVER}_mission_options {{}}")
    check("the model sees Feurisson's instructions", any("Feurisson" in c["system"] for c in seen))
    offered = {t for c in seen for t in c["tools"]}
    check("the model is offered the link's six tools, nothing else", offered == TOOLS, sorted(offered))
    check("mission_options answers through Open WebUI",
          any("barbecue" in r for c in seen for r in c["tool_results"]))
    check("the model receives the saved key", seen and all(c["authorization"] == f"Bearer {key}" for c in seen))
    check("routing: data_collection deny", seen and all(c["provider"] == DENY for c in seen),
          [c["provider"] for c in seen])
    direct = ask(token, "Hello", model=MODEL)
    check("the plain model, chosen by its id: routed the same way, no tools",
          direct and all(c["provider"] == DENY and not c["tools"] for c in direct),
          [(c["provider"], c["tools"]) for c in direct])

    again = session("start")
    check("a second start is refused", again.returncode != 0 and "already runs" in again.stderr, again.stderr[-300:])
    now = status()
    check("… and the session still runs", now["simulator"] and now["link"] and now["open_webui"].startswith("Up"), now)

    filters = {f["id"]: f for f in api("GET", "/functions/", token=token)}
    bot = api("GET", f"/models/model?id={BOT}", token=token) or {}
    check("the filter is installed, on, and Feurisson's", (filters.get(FILTER) or {}).get("is_active") is True
          and FILTER in ((bot.get("meta") or {}).get("filterIds") or []), [list(filters), bot.get("meta")])
    seen = ask(token, f'@call {SERVER}_start_mission {{"steps": [{{"do": "reach_card", "card": "barbecue"}}, '
                      f'{{"do": "set_light", "color": "green"}}]}}')
    check("the mission starts", any('"started"' in r for c in seen for r in c["tool_results"]),
          [r[:200] for c in seen for r in c["tool_results"]])
    seen = ask(token, f"@call {SERVER}_mission_status {{}} @call {SERVER}_mission_status {{}}")
    told = [r for c in seen for r in c["tool_results"]]
    check("mission_status read twice in one answer while the mission runs: the second says to tell the group now",
          len(told) == 2 and "a moment ago" not in told[0] and "a moment ago" in told[1], told)
    n0 = len(NAMED)
    seen = ask(token, "Bonjour Feurisson, il en est où le robot ?")
    system = " ".join(c["system"] for c in seen)
    check("the filter: the group's language, named by the small model, in the instructions",
          "The group writes in French" in system and NAMED[n0:]
          and all(n["provider"] == DENY for n in NAMED[n0:]), [system[-600:], NAMED[n0:]])
    check("the filter: the mission as it is now, in the instructions",
          "mission_status, read at" in system and '"state": "running"' in system, system[-600:])
    seen = ask(token, f"@call {SERVER}_pause_mission {{}} @call {SERVER}_stop_mission {{}}")
    told = [r for c in seen for r in c["tool_results"]]
    check("a stop right after a pause in the same answer is refused: the group is asked first",
          len(told) == 2 and "ask the group first" in told[1], told)
    seen = ask(token, f"@call {SERVER}_resume_mission {{}}")
    check("… and the mission goes on", any('"running"' in r for c in seen for r in c["tool_results"]),
          [r[:200] for c in seen for r in c["tool_results"]])
    seen = ask(token, f"@call {SERVER}_stop_mission {{}}")
    told = [r for c in seen for r in c["tool_results"]]
    check("a stop the group did not ask for, without a pause first, is refused", told and "pause_mission first" in told[0],
          told)
    report, t1 = "", time.monotonic()
    while time.monotonic() - t1 < 400:
        report = " ".join(r for c in ask(token, f"@call {SERVER}_mission_status {{}}") for r in c["tool_results"])
        if any(f'"state": "{s}"' in report for s in ("ended", "stopped", "failed")):
            break
        time.sleep(10)
    check("the mission ends, done, its report back through Open WebUI",
          '"state": "ended"' in report and '"outcome": "done"' in report, report[-400:])

    files = [p for p in (GUIDE / ".robot-sim").rglob("*") if p.is_file()]
    check("the key is in no file of .robot-sim", not any(key.encode() in p.read_bytes() for p in files),
          f"{len(files)} files")
    lines = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True).stdout
    check("the key is on no command line", key not in lines)


def gone(name):
    stopped = session("stop")
    check(f"{name}: robot-session stop", stopped.returncode == 0, stopped.stderr[-300:])
    left = [x[0] + " " + x[2] for x in (["container", "inspect", OWUI], ["volume", "inspect", VOLUME],
                                         ["network", "inspect", NETWORK])
            if subprocess.run(["docker", *x], capture_output=True).returncode == 0]
    check(f"{name}: Open WebUI, its conversations and its network are gone", not left, left)
    ps = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True).stdout
    check(f"{name}: no process of the session is left",
          "sim/robot_sim.py --port 8780" not in ps and "stack/interpreter_link.py" not in ps)


def after_restart(env):
    """A Codespace stopped without robot-session stop, then started again: pids.json names programs that are gone
    (their numbers can now be others), and Docker still has the old Open WebUI and its volume."""
    other = subprocess.Popen(["sleep", "300"], start_new_session=True)
    try:
        stat = Path(f"/proc/{other.pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
        PIDS.parent.mkdir(parents=True, exist_ok=True)
        PIDS.write_text(json.dumps({"simulator": {"pid": other.pid, "start": stat},
                                    "link": {"pid": other.pid, "start": "0"}}))
        now = status()
        check("after a restart: another program with a pid of pids.json is not the session",
              not now["simulator"] and not now["link"], now)
        subprocess.run(["docker", "volume", "create", VOLUME], capture_output=True)
        made = subprocess.run(["docker", "create", "--name", OWUI, "-v", f"{VOLUME}:/app/backend/data", OWUI_IMAGE],
                              capture_output=True, text=True)
        check("after a restart: an old Open WebUI is left in Docker", made.returncode == 0, made.stderr[-300:])
        started = subprocess.run([SESSION_CMD, "start"], env=env, capture_output=True, text=True)
        check("… and start still goes through", started.returncode == 0 and "Ready" in started.stdout,
              (started.stdout + started.stderr)[-600:])
        gone("second session")
        check("… and the other program was never stopped", other.poll() is None)
    finally:
        other.kill()


def main():
    wp = provider()
    key = wp.read_key()
    if key and not key.startswith("sk-or-"):
        sys.exit("check_session: the saved key is not an OpenRouter key, and robot-session needs one. Run this check "
                 "where workshop-login saved an sk-or- key, or none; it never replaces a key of yours.")
    if key and wp.europe():
        sys.exit("check_session: workshop-login --europe is on, and robot-session refuses it for now.")
    fake_key, model = not key, None
    try:
        if fake_key:
            key = "sk-or-v1-" + "0" * 64
            subprocess.run(["workshop-key", "save"], input=key, text=True, check=True)
        subprocess.run(["docker", "network", "create", NETWORK], capture_output=True)
        gateway = subprocess.run(["docker", "network", "inspect", NETWORK, "-f", "{{(index .IPAM.Config 0).Gateway}}"],
                                 capture_output=True, text=True).stdout.strip()
        check("Docker's network for the session, with a gateway", gateway, "no gateway")
        if gateway:
            model = ThreadingHTTPServer((gateway, FAKE_PORT), FakeModel)
            threading.Thread(target=model.serve_forever, daemon=True).start()
            env = dict(os.environ, ROBOT_STRATEGIST="none", ROBOT_MODEL=MODEL, ROBOT_LANGUAGE_MODEL="fake-small",
                       ROBOT_OPENAI_BASE_URL=f"http://{gateway}:{FAKE_PORT}/v1")
            try:
                first_session(key, gateway, env)
            finally:
                gone("first session")
            subprocess.run(["docker", "network", "create", NETWORK], capture_output=True)  # the fake model's address
            after_restart(env)
            check("nobody waits for an adult after stop", not (GUIDE / ".robot-sim" / "waiting.json").exists())
    finally:
        session("stop")
        if model:
            model.shutdown()
        if fake_key:
            subprocess.run(["sudo", "-n", "rm", "-f", str(Path.home() / ".workshop-key")])
        print(f"{RESULTS.count(True)} PASS / {RESULTS.count(False)} FAIL", flush=True)
    sys.exit(0 if all(RESULTS) else 1)


if __name__ == "__main__":
    main()
