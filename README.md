# Roborock Controller

Xbox-Controller (oder Tastatur) als Fernsteuerung für Roborock-Saugroboter (z. B. Qrevo Curv).  
Standard: nur fahren — Saugen und Wischen aus.

## Setup

```bash
git clone https://github.com/kanki66/roborock-controller.git
cd roborock-controller
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
chmod +x fahren
```

## Start

```bash
./fahren
```

Beim ersten Mal: Roborock-E-Mail und Code aus der Mail eingeben.  
Die Anmeldung wird lokal in `~/.config/roborock-xbox/session.json` gespeichert (nicht committen).

USB-Xbox-Controller ist zuverlässiger als Bluetooth. Optional: `./fahren --device curv`

## Steuerung

| Eingabe | Funktion |
|--------|----------|
| Linker Stick / WASD | Fahren |
| Q / E | Drehen (Tastatur) |
| LB / Shift | Langsam |
| X / V | Saugen an/aus |
| View / M | Wischen an/aus |
| A / Leertaste | Fernsteuerung starten |
| Y | Zur Station (Programm bleibt an) |
| B / Esc | Programm beenden |

## Hinweise

- Sturzsensoren bleiben aktiv.
- Livebild der Kamera ist hier nicht enthalten (nur über die Roborock-App / WebRTC).
- Andere Controller: aktuell auf Xbox ausgelegt.
