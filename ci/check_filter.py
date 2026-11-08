"""The filter of Open WebUI (interpreter/feurisson_filter.py) takes the personal details out of the group's messages
before they leave the Codespace. Checked here without Open WebUI, without a model, without a network:

    python3 -I ci/check_filter.py

- what a young person may say about themselves, in the languages of the partners: the detail is gone;
- what a group says to the robot: unchanged;
- a name found once is taken out of the later messages, and of the earlier ones, as Open WebUI sends them all again;
- what the patterns are known to miss is printed, not failed: the filter is a help, not a guarantee.
"""
import importlib.util
import sys
from pathlib import Path

GUIDE = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("feurisson_filter", GUIDE / "interpreter" / "feurisson_filter.py")
flt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flt)

# (message, the details that must be gone)
PERSONAL = [
    ("Je m'appelle Léa, j'ai 13 ans et j'habite rue des Pins.", ["Léa", "13", "rue des Pins"]),
    ("My name is Tom, I am 12 and I live on Elm Street.", ["Tom", "12", "Elm Street"]),
    ("je m'appelle léa", ["léa"]),
    ("Moi c'est Inès !", ["Inès"]),
    ("Mon prénom c'est Noah et j'ai 14 ans", ["Noah", "14"]),
    ("Je suis Camille.", ["Camille"]),
    ("Salut, je suis Jean Dupont", ["Jean", "Dupont"]),
    ("Hi, I'm Amelia.", ["Amelia"]),
    ("I'm 13 years old", ["13"]),
    ("I am 15.", ["15"]),
    ("my name's Jack and i live in Portsmouth", ["Jack", "Portsmouth"]),
    ("I go to Saint Mary school, it's nice", ["Saint Mary"]),
    ("Mon école c'est le collège Fromentin", ["Fromentin"]),
    ("J'habite à La Rochelle, près du port", ["La Rochelle"]),
    ("Me llamo Lucía y tengo 12 años", ["Lucía", "12"]),
    ("Vivo en Sabadell", ["Sabadell"]),
    ("Em dic Pau i tinc 13 anys", ["Pau", "13"]),
    ("Visc a Terrassa", ["Terrassa"]),
    ("Mi chiamo Giulia e ho 14 anni", ["Giulia", "14"]),
    ("Abito a Napoli, vicino al mare", ["Napoli"]),
    ("Ik heet Daan en ik ben 12 jaar", ["Daan", "12"]),
    ("Ik woon in Antwerpen", ["Antwerpen"]),
    ("Mijn naam is Lotte", ["Lotte"]),
    ("Mon mail c'est lea.martin@example.org", ["lea.martin@example.org"]),
    ("appelle-moi au 06 12 34 56 78", ["06 12 34 56 78"]),
    ("Elle a 13 ans", ["13"]),
]
# The whole message, as the model gets it: the words around the detail stay.
EXACT = [
    ("Je m'appelle Léa, j'ai 13 ans et j'habite rue des Pins.", "Je m'appelle [name], j'ai [age] et j'habite [place]."),
    ("My name is Tom, I am 12 and I live on Elm Street.", "My name is [name], I am [age] and I live [place]."),
    ("Ik heet Daan en ik ben 12 jaar", "Ik heet [name] en ik ben [age]"),
    ("Mi chiamo Giulia e ho 14 anni", "Mi chiamo [name] e ho [age]"),
    ("I'm Tom and I want the barbecue", "I'm [name] and I want the barbecue"),
]
ROBOT = [
    "Est-ce que le robot peut aller jusqu'à la caserne ?", "Can the robot go to the fire station?",
    "Cherche le barbecue, doucement.", "Look for the barbecue, carefully.", "Trouve toutes les cartes « herbe sèche ».",
    "Find every « dry grass » card.", "Oui !", "Yes, that's it.", "D'accord, vas-y.", "OK, go.",
    "Non, plutôt le point d'eau.", "No, the water point instead.", "Et regarde aussi la caserne !",
    "Wait, go to the barbecue instead.", "Non, laisse tomber, continue.", "Il en est où ?", "How is it going?",
    "Alors, ça a marché ?", "Raconte-nous une blague.", "What is the capital of Italy?", "Fonce dans la balle !",
    "Drive off the mat!", "Va jusqu'à la piscine.", "I'm ready!", "Je suis prêt.", "C'est parti !", "I'm here.",
    "Avance de 4 cases.", "Move 2 cells to the left.", "Allume la lumière en vert.", "Il peut aller vers l'eau ?",
    "Go where there is fire.", "Fais-le bouger, vite !", "Is there any grass?", "Ve a la estación de bomberos.",
    "Vés a la caserna de bombers.", "Vai al barbecue, piano.", "Ga naar het waterpunt.", "Stop!", "Arrête le robot.",
    "Le robot est sur la case E5 ?", "Il est où le robot, en C3 ?", "On veut la personnalité prudente.",
    "Choose the adventurous one.", "Il y a 3 cartes herbe sèche ?", "C'est Feurisson ?", "Je suis Feurisson, le robot ?",
    "I am 2 cells from the barbecue", "Go 3 cells and stop", "I'm 12 cells away, I think", "J'ai 10 cases à faire",
]
# Known to be missed by the patterns: printed, not failed.
KNOWN_MISSES = [
    ("Léa et Tom veulent aller au barbecue", ["Léa", "Tom"]),
    ("C'est Tom qui a choisi", ["Tom"]),
    ("We are Mia and Leo from class 5B", ["Mia", "Leo"]),
    ("je suis en 4e au collège", ["4e"]),
]
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
        print(f"PASS  {name}")
    else:
        failed += 1
        print(f"FAIL  {name}  {detail}")


for text, details in PERSONAL:
    masked, _ = flt.mask(text)
    left = [d for d in details if d in masked]
    check(f"taken out: {text}", not left and "[" in masked, f"-> {masked!r}, left {left}")
for text, expected in EXACT:
    masked, _ = flt.mask(text)
    check(f"exactly: {text}", masked == expected, f"-> {masked!r}")
for text in ROBOT:
    masked, names = flt.mask(text)
    check(f"unchanged: {text}", masked == text, f"-> {masked!r}")

# Across a conversation, as Open WebUI sends it at each message of the group.
f = flt.Filter()
f.turn_hook = lambda asked: '{"state": "no mission yet"}'
body = {"messages": [{"role": "system", "content": "Feurisson"},
                     {"role": "user", "content": "Léa veut aller au barbecue"},
                     {"role": "assistant", "content": "OK?"},
                     {"role": "user", "content": "Je m'appelle Léa"},
                     {"role": "assistant", "content": "Hello!"},
                     {"role": "user", "content": [{"type": "text", "text": "Léa dit oui"}]}]}
out = f.inlet(body, {"chat_id": "c1"})
users = [m["content"] if isinstance(m["content"], str) else m["content"][0]["text"]
         for m in out["messages"] if m["role"] == "user"]
check("a name found in one message is taken out of the earlier and later ones",
      all("Léa" not in u for u in users), users)
check("the instructions say what the brackets mean", "[name]" in out["messages"][0]["content"])
check("another conversation keeps its own names", f.inlet({"messages": [{"role": "user", "content": "Léa"}]},
                                                          {"chat_id": "c2"})["messages"][-1]["content"] == "Léa")

answer = {"messages": [{"role": "user", "content": "x"},
                       {"role": "assistant", "content": "Bonjour [name] ! Tu as [age], super. Hi [name], let's go."}]}
check("the brackets written back by the model are taken out of its answer",
      f.outlet(answer)["messages"][-1]["content"] == "Bonjour ! Tu as, super. Hi, let's go.",
      answer["messages"][-1]["content"])

for text, details in KNOWN_MISSES:
    masked, _ = flt.mask(text)
    print(f"KNOWN {'missed' if any(d in masked for d in details) else 'caught'}: {text!r} -> {masked!r}")
print(f"{passed} PASS / {failed} FAIL")
sys.exit(1 if failed else 0)
