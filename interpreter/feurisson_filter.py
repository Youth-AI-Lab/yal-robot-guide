"""
title: Feurisson robot
author: Youth AI Lab
description: Before each message of the group, takes out the names, ages and places a young person gives, adds to Feurisson's instructions the mission as it is now and the group's language (named by a small model), and tells the robot's link that a new turn starts.
version: 2
"""
# An Open WebUI filter (Functions), installed on the chatbot "Feurisson robot" by bin/robot-session. On 7 and
# 8 November 2026, DeepSeek v4 Flash as Feurisson answered in French to groups writing in English (about 13 % of the
# turns), stopped a mission without asking the group (4 turns in 198), and once told an old state of the mission:
# rewording its instructions did not help. Here the
# filter does it, outside the model, before each message of the group:
#   - the state of the mission, read now from the link (POST /turn), goes into the instructions;
#   - the link learns that a new turn starts: it then refuses a stop_mission that follows a pause_mission in the same
#     answer, unless the group asked to stop (stack/interpreter_link.py);
#   - a small, cheap model names the group's language; the last one named stays for the conversation, written in the
#     instructions;
#   - first of all, a name, an age, a place where someone lives, a school, an e-mail or a phone number in the group's
#     messages is replaced by [name], [age], [place], [school] or [contact], with patterns, without any model: the
#     detail never leaves the Codespace (Open WebUI keeps the message as typed, in its own base). Asked to say that it
#     does not keep such details, or to refuse what is off its topic, DeepSeek greeted the young person by the name
#     5 or 6 times out of 13 (8 November 2026). The patterns miss
#     what they do not know (a name given without "my name is", a nickname): a help, not a guarantee.
# Nothing is kept on disk; the languages and the names found stay in memory, per conversation, while Open WebUI runs.
# The group's message, once the details are taken out, goes to the small model (the same provider and the same rules
# as Feurisson's model) and to no one else.
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
# What a young person may say about themselves, in the languages of the partners (French, English, Spanish, Catalan,
# Italian, Dutch). Each pattern keeps the words before the detail and replaces the detail.
WORD = r"[A-Za-zÀ-ÖØ-öø-ÿ'’-]+"
NAME = rf"(?:[A-ZÀ-ÖØ-Þ]{WORD}(?:\s+[A-ZÀ-ÖØ-Þ]{WORD})?|{WORD})"
CLAUSE = r"[^,.;!?\n]+?(?=\s+(?:et|and|y|i|e|en)\s|[,.;!?\n]|$)"
PERSONAL = [
    ("name", re.compile(rf"(\b(?i:je m'appelle|je m’appelle|mon nom (?:c'est|est)|mon prénom (?:c'est|est)|moi c'est|"
                        rf"my name is|my name's|i'm called|i am called|me llamo|mi nombre es|em dic|el meu nom "
                        rf"és|mi chiamo|il mio nome è|ik heet|mijn naam is)\s+)({NAME})")),
    ("name", re.compile(rf"(\b(?i:I'm|I am|I’m|je suis|c'est|soy|sóc|sono|ik ben)\s+)([A-ZÀ-ÖØ-Þ][a-zà-öø-ÿ'’-]+"
                        rf"(?:\s+[A-ZÀ-ÖØ-Þ][a-zà-öø-ÿ'’-]+)?)(?=\s*(?:[,.;!?\n]|$|\s+(?:et|and|y|i|e|en)\s))")),
    ("age", re.compile(r"(\b(?:j'ai|j’ai|i am|i'm|i’m|tengo|tinc|ho|ik ben)\s+)(\d{1,2}(?!\d)(?:\s*(?:ans|an|years?(?:\s+old)?|"
                       r"y/?o|años|anys|anni|jaar)\b)?)(?!\s*(?:cells?|cases?|squares?|steps?|pas\b|cm|m\b|min|s\b|sec|fois|times?|"
                       r"casillas?|caselle|vakjes?|%))", re.I)),
    ("age", re.compile(r"()(\b\d{1,2}\s*(?:ans|years old|años|anys|anni|jaar)\b)", re.I)),
    ("place", re.compile(rf"(\b(?:j'habite|j’habite|je vis|i live|vivo|visc|abito|ik woon)\s+)({CLAUSE})", re.I)),
    ("school", re.compile(rf"(\b(?:mon école|mon collège|mon lycée|ma classe|my school|i go to|mi escuela|mi colegio|"
                          rf"la meva escola|la mia scuola|mijn school)\s*(?:c'est|est|s'appelle|is|es|és|è|is called|"
                          rf"heet)?\s*)({CLAUSE})", re.I)),
    ("contact", re.compile(r"()([\w.+-]+@[\w-]+\.[\w.-]+|\+?\d[\d .-]{7,}\d)")),
]
BRACKETS = re.compile(r"\s*\[(?:name|age|place|school|contact)\]")
# Words after "I'm" or "je suis" that are not names, so that "I'm Ready" or "C'est Parti" stay as typed.
NOT_NAMES = {"ok", "okay", "ready", "here", "back", "sure", "sorry", "done", "prêt", "prête", "parti", "partie", "bon",
             "bien", "fini", "the", "le", "la", "les", "un", "une", "a", "an", "it", "ça", "pas", "not", "feurisson",
             "robot"}


def mask(text, known=()):
    """The text with the personal details replaced, and the names found (to take out of later messages too)."""
    names = set()

    def replace(kind):
        def one(m):
            detail = m.group(2)
            if kind == "name":
                if detail.split()[0].lower() in NOT_NAMES:
                    return m.group(0)
                names.add(detail.split()[0])
            return m.group(1) + f"[{kind}]"
        return one

    for kind, pattern in PERSONAL:
        text = pattern.sub(replace(kind), text)
    for name in set(known) | names:
        if len(name) > 1:
            text = re.sub(rf"\b{re.escape(name)}\b", "[name]", text)
    return text, names


# The commonest short words of each language of the partners: a guess, used only when the small model names nothing.
COMMON = {
    "French": {"le", "la", "les", "un", "une", "des", "est", "et", "je", "tu", "il", "on", "va", "pas", "du", "au",
               "avec", "pour", "que", "qui", "où", "oui", "non", "robot", "carte", "cherche", "trouve", "aller"},
    "English": {"the", "a", "an", "is", "and", "i", "you", "it", "go", "to", "not", "of", "with", "for", "what",
                "where", "yes", "no", "can", "find", "look", "card", "my", "am", "we", "are"},
    "Spanish": {"el", "la", "los", "las", "un", "una", "es", "y", "yo", "tú", "va", "no", "del", "al", "con", "para",
                "qué", "dónde", "sí", "busca", "encuentra", "ve", "mi", "soy", "tengo"},
    "Catalan": {"el", "la", "els", "les", "un", "una", "és", "i", "jo", "tu", "va", "no", "del", "al", "amb", "per",
                "què", "on", "sí", "busca", "troba", "vés", "em", "sóc", "tinc"},
    "Italian": {"il", "lo", "la", "gli", "le", "un", "una", "è", "e", "io", "tu", "va", "non", "del", "al", "con",
                "per", "che", "dove", "sì", "cerca", "trova", "vai", "mi", "sono", "ho"},
    "Dutch": {"de", "het", "een", "is", "en", "ik", "jij", "je", "ga", "naar", "niet", "van", "met", "voor", "wat",
              "waar", "ja", "nee", "zoek", "vind", "kaart", "mijn", "ben"},
}


def guess_language(text):
    """The language whose common words the message uses most, when it leads by two words at least; else None."""
    words = re.findall(r"[a-zà-ÿ']+", text.lower().replace("’", "'"))
    counts = sorted(((sum(w in common for w in words), name) for name, common in COMMON.items()), reverse=True)
    return counts[0][1] if counts[0][0] >= 2 and counts[0][0] - counts[1][0] >= 2 else None


NAMING = ("Name the language the user's message is written in, in English, in one word (for example French, English, "
          "Spanish, Catalan, Italian, Dutch). If the message is too short to tell, answer: unknown.")


class Filter:
    class Valves(BaseModel):
        language_model: str = Field(default="", description="the small model that names the group's language")
        language_extra: str = Field(default="", description="JSON added to its requests (OpenRouter's routing)")

    def __init__(self):
        self.valves = self.Valves()
        self.languages = {}  # conversation -> the language named last
        self.names = {}  # conversation -> the names found in its messages, taken out of the later ones too
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

    def take_out(self, messages, chat):
        """Replace the personal details in every message of the group: Open WebUI sends the whole conversation, as
        typed, each time."""
        known = self.names.setdefault(chat, set())
        if len(self.names) > 500:
            self.names.clear()
            self.names[chat] = known
        for m in messages:
            if m.get("role") != "user":
                continue
            if isinstance(m.get("content"), str):
                m["content"], found = mask(m["content"], known)
                known |= found
            elif isinstance(m.get("content"), list):
                for part in m["content"]:
                    if isinstance(part, dict) and isinstance(part.get("text"), str):
                        part["text"], found = mask(part["text"], known)
                        known |= found
        if known:  # a name found later is taken out of the earlier messages too
            for m in messages:
                if m.get("role") == "user" and isinstance(m.get("content"), str):
                    m["content"], _ = mask(m["content"], known)

    def inlet(self, body, __metadata__=None):
        messages = body.get("messages") or []
        chat = (__metadata__ or {}).get("chat_id") or "-"
        self.take_out(messages, chat)
        said = next((m.get("content") for m in reversed(messages) if m.get("role") == "user"), "")
        said = said if isinstance(said, str) else " ".join(p.get("text", "") for p in said if isinstance(p, dict))
        if len(said.split()) >= 2:
            # The small model sometimes names nothing (1 conversation in 80 on 8 November 2026, answered in French to
            # a group writing in English from its first message on): ask once more, then guess from the common words.
            named = self.name_language(said) or self.name_language(said) or guess_language(said)
            if named:
                if len(self.languages) > 500:
                    self.languages.clear()
                self.languages[chat] = named
        status = self.group_turn(bool(STOP_ASK.search(said)))
        lines = ["- The group's messages reach you with names, ages and places replaced by [name], [age], [place], "
                 "[school] or [contact]. Never write these brackets, never ask for what they replace."]
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

    def outlet(self, body, __metadata__=None):
        """The brackets of take_out written back by the model ("Bonjour [name] !", 3 answers in 160 conversations on
        8 November 2026) are taken out of its answer before the group reads it."""
        for m in (body.get("messages") or [])[-1:]:
            if m.get("role") == "assistant" and isinstance(m.get("content"), str):
                m["content"] = BRACKETS.sub("", m["content"])
        return body
