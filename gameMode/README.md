# GameMode Drohnensteuerung

Das Modul **`gameMode`** bietet eine intuitive, spiele-engine-artige Abstraktionsschicht (**"Character Controller / Game Model"**) zur Ansteuerung einer ArduPilot-Drohne im **`ALT_HOLD`**-Modus (oder `FLOWHOLD`) **komplett ohne GPS**.

---

## 📁 Struktur des Ordners

* [`actor.py`](file:///c:/Users/User/Documents/GitHub/ai-drone/gameMode/actor.py): Kern-Implementierung
  * `Vector3`: 3D-Vektor-Hilfsklasse für Richtungs- und Geschwindigkeitsvorgaben.
  * `DroneGameActor`: Die Haupt-Steuerklasse für Start, Landung, Richtungs- und Vektorbewegungen.
  * `FailsafeConfig` & `FailsafeException`: Konfiguration und Ausnahmebehandlung des Sicherheitswächters.
* [`demo.py`](file:///c:/Users/User/Documents/GitHub/ai-drone/gameMode/demo.py): Interaktives Beispielskript mit zwei Betriebsmodi:
  * **Choreografie:** Automatisierte Flugsequenz (Start $\rightarrow$ Vorwärts $\rightarrow$ Drehung $\rightarrow$ Landung).
  * **Tastatur-Controller:** Echtzeit-Steuerung über Terminal-Tasten (WASD / Space / C / Q / E / X / K).
* [`__init__.py`](file:///c:/Users/User/Documents/GitHub/ai-drone/gameMode/__init__.py): Package-Initialisierung und Export der Hauptklassen.

---

## 🕹️ Wie funktioniert die Steuerung?

Die Steuerung erfolgt im **Body-Frame** (aus Sicht der Flug- und Kamerarichtung der Drohne):

```text
               [ Vorwärts: Pitch 1380-1450 ]
                             ▲
                             │
[ Links: Roll 1380-1450 ] ◄──┼──► [ Rechts: Roll 1550-1620 ]
                             │
                             ▼
              [ Rückwärts: Pitch 1550-1620 ]

[ Steigen: Throttle 1550-1650 ]   |   [ Sinken: Throttle 1350-1450 ]
[ Drehung Links: Yaw 1380-1450 ]  |   [ Drehung Rechts: Yaw 1550-1620 ]
[ Neutral / Schwebeflug: Alle Kanäle auf 1500 PWM ]
```

1. **`ALT_HOLD` Modus:** ArduPilot hält die Höhe automatisch über den LiDAR-Sensor, solange das Gas auf Neutral ($1500\text{ PWM}$) steht.
2. **20 Hz Hintergrund-Streamer:** Ein Hintergrund-Thread (`_streamer_loop`) sendet kontinuierlich MAVLink RC-Overrides mit 20 Hz, sodass die Steuerungsverbindung niemals abbricht.
3. **Befehl aktiv (z. B. `drone.move_forward()`):** Drohne neigt sich nach vorne und fliegt.
4. **Befehl beendet / `drone.hover()` / `drone.stop()`:** Alle Achsen springen sofort auf $1500\text{ PWM}$ (Neutral) $\rightarrow$ Die Drohne richtet sich waagerecht aus und schwebt stabil auf der aktuellen Höhe.

---

## 🚨 Integriertes Failsafe-System

Der Actor überwacht im Flug kontinuierlich vier Sicherheitsgrenzen:

| Failsafe-Wächter | Grenzwert (Standard) | Reaktion bei Überschreitung |
| :--- | :--- | :--- |
| **Höhenlimit** | `max_altitude = 0.80 m` (80 cm) | Sofortiger Motorstopp & Force Disarm |
| **Vertikalgeschwindigkeit** | `max_vertical_speed = 0.80 m/s` | Sofortiger Motorstopp & Force Disarm |
| **Schräglage / Überschlag** | `max_tilt_angle_deg = 25.0°` | Sofortiger Motorstopp & Force Disarm |
| **Sensor-Signalverlust** | `telemetry_timeout_s = 0.50 s` | Sofortiger Motorstopp & Force Disarm |
| **Not-Aus (Kill Switch)** | Taste `K`, `ESC` oder `emergency_kill()` | 1000 PWM + MAVLink Force Disarm (`21196`) |

---

## 💻 Code-Beispiele

### 1. Einfache Bewegungsmethoden
```python
from gameMode import DroneGameActor, Vector3

with DroneGameActor(device="/dev/serial0", baud=115200) as drone:
    # 1. Start auf 50 cm Höhe
    drone.takeoff(height_m=0.50)

    # 2. Richtungsbefehle
    drone.move_forward(duration_s=1.0, speed=0.35)
    drone.move_right(duration_s=1.0, speed=0.35)
    drone.rotate_yaw(duration_s=1.0, speed=0.40)  # Rechtsdrehung

    # 3. Schwebeflug
    drone.hover(duration_s=2.0)

    # 4. Sichere Landung mit Touchdown-Abschaltung (<= 5 cm)
    drone.land()
```

### 2. Vektorbewegung mit `Vector3`
```python
# Diagonale Bewegung: Vorwärts + Links
move_dir = Vector3.forward() + Vector3.left()
drone.move(vector=move_dir, duration_s=1.5, speed=0.4)
```

### 3. Kontinuierliche Eingaben (Game-Loop / WASD / Gamepad / KI)
```python
# Werte jeweils normiert von -1.0 bis +1.0:
drone.set_axis_input(forward=0.5, strafe=0.0, vertical=0.0, yaw=0.0)

# Zum Anhalten und Schweben:
drone.stop()
```

---

## 🚀 Ausführung des Demo-Skripts

### Simulation / Dry-Run (ohne echte Drohne / MAVLink-Hardware)
```bash
# Automatisierte Choreografie
uv run python -m gameMode.demo --dry-run --demo choreo

# Interaktive Tastatur-Steuerung (WASD)
uv run python -m gameMode.demo --dry-run --demo keyboard
```

### Echter Flugbetrieb (auf dem Raspberry Pi)
```bash
uv run python -m gameMode.demo --device /dev/serial0 --baud 115200 --demo choreo
```

---

## 🧪 Tests ausführen
```bash
uv run pytest -q tests/test_game_mode.py
```
