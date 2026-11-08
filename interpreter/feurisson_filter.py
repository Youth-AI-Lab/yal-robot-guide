"""
title: Feurisson robot
author: Youth AI Lab
description: Before each message of the group, adds to Feurisson's instructions the mission as it is now and the group's language (named by a small model), and tells the robot's link that a new turn starts.
version: 1
"""
# An Open WebUI filter (Functions), installed on the chatbot "Feurisson robot" by bin/robot-session. On 7 and
# 8 November 2026, DeepSeek v4 Flash as Feurisson answered in French to groups writing in English (about 13 % of the
# turns), stopped a mission without asking the group (4 turns in 198), and once told an old state of the mission:
# rewording its instructions did not help (facilitator/workshops/measures/2026-11-08-robot-feurisson-gaps/). Here the
# filter does it, outside the model, before each message of the group:
#   - the state of the mission, read now from the link (POST /turn), goes into the instructions;
#   - the link learns that a new turn starts: it then refuses a stop_mission that follows a pause_mission in the same
#     answer, unless the group asked to stop (stack/interpreter_link.py);
#   - a small, cheap model names the group's language; the last one named stays for the conversation, written in the
#     instructions.
# Nothing is kept on disk; the languages stay in memory, per conversation, while Open WebUI runs. The group's
# message goes to the small model (the same provider and the same rules as Feurisson's model) and to no one else.
# ci/probe_interpreter.py --filter loads this file too, to measure what it changes.
import json
import os
import re
import time
import urllib.request

try:
    from pydantic import BaseModel, Field
except ImportError:  # the probe, outside Open WebUI
    def Field(default=None, **kwargs):
        return default

    class BaseModel:
        def __init__(self, **values):
            for name in dir(type(self)):
                if not name.startswith("_"):
                    setattr(self, name, values.get(name, getattr(type(self), name)))

MARK = "\n\n## Right now (from the robot, not from the group)\n"
# The group asks to stop, in the languages of the partners: then a pause and a stop in the same answer are right.
STOP_ASK = re.compile(r"\b(stop|stoppe?|arrête|arrêter|annule|cancel|abort|para|detente|cancela|atura|fermati|ferma|"
                      r"basta|annulla|halt|annuleer)\b|laisse tomber la mission", re.I)
NAMING = ("Name the language the user's message is written in, in English, in one word (for example French, English, "
          "Spanish, Catalan, Italian, Dutch). If the message is too short to tell, answer: unknown.")


class Filter:
    class Valves(BaseModel):
        language_model: str = Field(default="", description="the small model that names the group's language")
        language_extra: str = Field(default="", description="JSON added to its requests (OpenRouter's routing)")

    def __init__(self):
        self.valves = self.Valves()
        self.languages = {}  # conversation -> the language named last
        # Set by the probe; in Open WebUI, read from the environment of the container.
        self.language_url, self.language_key, self.turn_hook = None, None, None

    def link(self):
        """The link's address and token, from the tool server declared to Open WebUI (TOOL_SERVER_CONNECTIONS)."""
        try:
            servers = json.loads(os.environ.get("TOOL_SERVER_CONNECTIONS") or "[]")
        except ValueError:
            return None
        for server in servers:
            if (server.get("info") or {}).get("id") == "robot-mission":
                return server["url"].rsplit("/mcp", 1)[0], server.get("key", "")
        return None

    def group_turn(self, asked_to_stop):
        """Tell the link a new turn starts; its answer is mission_status, read now. None when it cannot be read."""
        if self.turn_hook is not None:
            return json.loads(self.turn_hook(asked_to_stop))
        found = self.link()
        if found is None:
            return None
        url, token = found
        request = urllib.request.Request(url + "/turn", data=json.dumps({"group_asked_to_stop": asked_to_stop}).encode(),
                                         headers={"Content-Type": "application/json",
                                                  "Authorization": "Bearer " + token})
        try:
            with urllib.request.urlopen(request, timeout=5) as r:
                return json.loads(r.read())
        except (OSError, ValueError):
            return None

    def name_language(self, text):
        """The language of the group's message, named by the small model; None when it cannot tell."""
        url = self.language_url or os.environ.get("OPENAI_API_BASE_URL", "")
        key = self.language_key or os.environ.get("OPENAI_API_KEY", "")
        if not (self.valves.language_model and url and key):
            return None
        body = {"model": self.valves.language_model, "max_tokens": 5, "temperature": 0,
                "messages": [{"role": "system", "content": NAMING}, {"role": "user", "content": text[:500]}]}
        try:
            body.update(json.loads(self.valves.language_extra or "{}"))
        except ValueError:
            pass
        request = urllib.request.Request(url.rstrip("/") + "/chat/completions", data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json", "Authorization": "Bearer " + key})
        try:
            with urllib.request.urlopen(request, timeout=15) as r:
                said = json.loads(r.read())["choices"][0]["message"]["content"] or ""
        except (OSError, ValueError, KeyError, IndexError):
            return None
        word = re.sub(r"[^A-Za-z]", "", (said.split() or [""])[0]).capitalize()
        return None if not word or word == "Unknown" else word

    def inlet(self, body, __metadata__=None):
        messages = body.get("messages") or []
        said = next((m.get("content") for m in reversed(messages) if m.get("role") == "user"), "")
        said = said if isinstance(said, str) else " ".join(p.get("text", "") for p in said if isinstance(p, dict))
        chat = (__metadata__ or {}).get("chat_id") or "-"
        if len(said.split()) >= 2:
            named = self.name_language(said)
            if named:
                if len(self.languages) > 500:
                    self.languages.clear()
                self.languages[chat] = named
        status = self.group_turn(bool(STOP_ASK.search(said)))
        lines = []
        language = self.languages.get(chat)
        if language:
            lines.append(f"- The group writes in {language}. Write every answer in {language} only, even when cards, "
                         "tools or reports use English words.")
        if status is None:
            lines.append("- mission_status could not be read just now: call it before saying how the mission goes.")
        else:
            lines.append(f"- mission_status, read at {time.strftime('%H:%M:%S')}, just before this message of the "
                         "group: " + json.dumps(status, ensure_ascii=False) + ". It may change while you answer: call "
                         "mission_status again before saying that something new happened.")
        block = MARK + "\n".join(lines)
        if messages and messages[0].get("role") == "system" and isinstance(messages[0].get("content"), str):
            messages[0]["content"] = messages[0]["content"].split(MARK)[0] + block
        else:
            messages.insert(0, {"role": "system", "content": block.lstrip()})
        body["messages"] = messages
        return body
