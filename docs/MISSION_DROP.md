# Autonome AprilTag Abwurf-Mission (`mission_drop.py`)

Dieses Dokument beschreibt die vollständige Architektur, die State-Machine-Ablaufdiagramme, das mathematische Suchmuster, den MT-15 Kollisionsschutz und die Bedienung der autonomen Abwurf-Mission für die **AI-Drone**.

---

## 1. Systemübersicht & Hardware-Architektur

Das System verbindet Computer Vision, Mehrsensor-Fusion und Aktuatorik auf dem Raspberry Pi Zero 2 W mit der Flugstabilisierung von ArduPilot:

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 Raspberry Pi Zero 2 W                  │
                  │                                                        │
                  │   ┌────────────────────────────────────────────────┐   │
                  │   │        mission_drop.py (State Machine)         │   │
                  │   └──────┬─────────────┬─────────────┬─────────────┘   │
                  └──────────┼─────────────┼─────────────┼─────────────────┘
                             │             │             │
                    CSI/I2C  │    GPIO 12  │    MAVLink  │ UART (/dev/serial0)
                             ▼             ▼             ▼
        ┌──────────────────────┐ ┌──────────┐ ┌────────────────────────────┐
        │  Raspberry Pi IMX500 │ │  SG90    │ │     Flywoo GN745 AIO       │
        │  AI-Kamera (Nadir)   │ │  Servo   │ │   (ArduPilot Copter 4.6)   │
        │  (AprilTag 3 / CV)   │ │ (Abwurf) │ └──────┬──────────────┬──────┘
        └──────────────────────┘ └──────────┘        │              │
                                            UART5    │     UART2    │
                                                     ▼              ▼
                                          ┌──────────────┐ ┌──────────────┐
                                          │ MicoAir      │ │ MicoAir      │
                                          │ MTF-01P      │ │ MT-15        │
                                          │ (Flow+LiDAR) │ │ (ToF Vorne)  │
                                          └──────────────┘ └──────────────┘
```

### Komponenten & Schnittstellen

1. **IMX500 AI-Kamera (Nadir, 90° nach unten):** Erfasst kontinuierlich hochauflösende Kamerabilder des Bodens zur Erkennung von AprilTag 3 (tag36h11) Markern.
2. **SG90 Servo (GPIO 12):** Löst den mechanischen Abwurfmechanismus über `gpiozero.Servo` aus.
3. **MicoAir MTF-01P (Boden):** Liefert optischen Fluss für die horizontale Drift-Stabilisierung und LiDAR-Höhendaten (`RNGFND1_ORIENT=25`).
4. **MicoAir MT-15 (Vorne):** ToF-Distanzmesser (bis 15 m) zur Echtzeit-Wanderkennung und Kollisionsvermeidung in der Flugrichtung (`RNGFND2_ORIENT=0`).
5. **Flywoo GN745 AIO Flight Controller:** Führt EKF3-Zustandsschätzung aus und empfängt Body-Frame Geschwindigkeitsbefehle (`SET_POSITION_TARGET_LOCAL_NED`).

---

## 2. Ablaufdiagramm der State Machine

Die gesamte Mission ist als deterministischer, fehlertoleranter Zustandsautomat aufgebaut:

```mermaid
flowchart TD
    Start([Skriptstart: ./mission_drop.py]) --> Init[Hardware-Init: Servo, Kamera, MAVLink]
    Init --> Takeoff[STATE_TAKEOFF: Steigflug auf Zielhöhe z.B. 0.6 m]
    
    Takeoff --> Search[STATE_SEARCH: 360° Expanding Square Suchspirale]
    
    Search --> CheckWall{Wand in < 1.0m erkannt?<br/>MT-15 Sensor}
    CheckWall -- Ja --> TurnLeg[Vorwärts-Schenkel abbrechen & abbiegen]
    TurnLeg --> Search
    CheckWall -- Nein --> CheckTag{AprilTag stabil?<br/>≥ 3 Frames, Hamming=0}
    
    CheckTag -- Nein --> CheckTimeout{Suchzeit > 90s?}
    CheckTimeout -- Ja --> Land[STATE_LAND: Geofence-Timeout Landung]
    CheckTimeout -- Nein --> Search
    
    CheckTag -- Ja --> Center[STATE_CENTER: P-Regler Zentrierung im Body-Frame]
    
    Center --> TagLost{Tag > 2.5s verloren?}
    TagLost -- Ja --> Search
    TagLost -- Nein --> CheckStable{Stabil zentriert?<br/>|dx, dy| < 35px für 1.8s}
    
    CheckStable -- Nein --> Center
    CheckStable -- Ja --> Drop[STATE_DROP: Schweben, Servo öffnen 1.5s, Servo schließen]
    
    Drop --> Land
    Land --> Finished([Drohne sicher am Boden / Disarmed])
    
    %% Failsafes
    BatteryFailsafe[Batterie < 14.4V] -.-> Land
    UserAbort[STRG + C Not-Aus] -.-> Land
```

---

## 3. Mathematisches Suchmuster: Expanding Square Spiral

Um von einem **beliebigen Startpunkt** in einer **unbekannten Halle** das Tag an einem **beliebigen Ort** sicher zu finden, nutzt das Skript eine wachsende quadratische Suchspirale:

$$\text{Dauer des Schenkels } n = \left\lceil \frac{n}{2} \right\rceil \times T_{\text{step}}$$

```
                       ▲ Vorwärts (3 x T)
                       │
       ┌───────────────┼───────────────────┐
       │               │       ┌───────┐   │
       │   ◄───────────┼───────┤ Start │   │   ► Rechts (1xT, 3xT...)
       │   Links (2xT) │       └───┬───┘   │
       │               │           │       │
       │               │           ▼       │
       └───────────────┼───────────────────┘
                       │
                       ▼ Rückwärts (2 x T, 4 x T...)
```

* **Schenkel 1:** $1 \times T$ Vorwärts ($+v_x$)
* **Schenkel 2:** $1 \times T$ Rechts ($+v_y$)
* **Schenkel 3:** $2 \times T$ Rückwärts ($-v_x$)
* **Schenkel 4:** $2 \times T$ Links ($-v_y$)
* **Schenkel 5:** $3 \times T$ Vorwärts ($+v_x$)
* **Schenkel 6:** $3 \times T$ Rechts ($+v_y$)
* **...**

### Warum ist das optimal?
* **360° Abdeckung:** Sucht gleichmäßig in alle 4 Himmelsrichtungen um den Startpunkt.
* **Lückenlose Erfassung:** Bei $T_{\text{step}} = 3.0\text{ s}$ und $v = 0.15\text{ m/s}$ beträgt der Versatz pro Schenkel $0.45\text{ m}$. Dies überlappt perfekt mit dem ca. $0.80\text{ m}$ breiten Kamerasichtfeld bei $0.60\text{ m}$ Flughöhe.

---

## 4. MT-15 Wand-Kollisionsschutz

Fliegt die Drohne im Suchmuster auf eine Hallenwand zu:

1. Der nach vorne gerichtete MT-15 ToF-Sensor misst die Distanz zur Wand in Echtzeit.
2. Unterschreitet der Abstand den Schwellenwert `--min-wall-dist` (Standard: `1.0 m`), wird der Vorwärtsflug **sofort gestoppt**.
3. Das Skript ruft `searcher.advance_to_next_leg()` auf $\rightarrow$ die Drohne **biegt sofort nach rechts ab** und setzt die Spirale im freien Bereich fort.
4. Auch während der Zentrierung (`STATE_CENTER`) wird $v_x$ bei Wandabständen $< 0.6\text{ m}$ auf `0.0` gekappt.

---

## 5. Zentrierungs-Regelung (P-Controller)

Die Kamera ist im Nadir-Modus (senkrecht nach unten) montiert:
* Bildbreite: $1280\text{ px}$, Bildhöhe: $960\text{ px}$
* Bildzentrum: $(x_c, y_c) = (640, 480)$

Die Soll-Geschwindigkeiten im Body-Frame berechnen sich zu:

$$v_x = (y_c - y_{\text{tag}}) \cdot k_p \quad (\text{Oben im Bild ist Vorwärts})$$
$$v_y = (x_{\text{tag}} - x_c) \cdot k_p \quad (\text{Rechts im Bild ist Rechts})$$

* Proportionalverstärkung: $k_p = 0.0012$
* Geschwindigkeitslimit: $|v_x|, |v_y| \le v_{\text{max}} = 0.20\text{ m/s}$
* **Abwurf-Kriterium:** $|dx| < 35\text{ px}$ und $|dy| < 35\text{ px}$ stabil für $\ge 1.8\text{ Sekunden}$.

---

## 6. Sicherheitswächter & Failsafes

| Sicherheitsmechanismus | Auslöser | Aktion |
| :--- | :--- | :--- |
| **Terminal Not-Aus** | Benutzer drückt `STRG + C` | `DroneController.__exit__` sendet sofort `LAND` / `DISARM` |
| **LiPo-Batteriewächter** | Akkuspannung $< 14.4\text{ V}$ | Sofortiger Abbruch & automatische Notlandung |
| **Höhenwächter** | Höhe $> 1.0\text{ m}$ (`--max-alt`) | Notlandung durch `DroneController` |
| **Suchzeit-Limit** | Suchzeit $> 90.0\text{ s}$ (`--max-search-time`) | Sanfte Landung am aktuellen Ort |
| **Zielverlust-Timeout** | Tag im Center-Modus $> 2.5\text{ s}$ weg | Rückkehr in den Suchmodus |
| **Tag-Rauschfilter** | Hamming $> 0$ oder Margin $< 25$ | Verwurf der Fehlmessung (mind. 3 Frames nötig) |

---

## 7. CLI-Parameter & Bedienung

### Standardstart auf dem Raspberry Pi:
```bash
cd ~/ai-drone
./mission_drop.py
```

### Konfigurations-Optionen:
```bash
./mission_drop.py \
  --takeoff-alt 0.6 \
  --max-alt 1.0 \
  --min-battery 14.4 \
  --min-wall-dist 1.0 \
  --pattern spiral \
  --step-time 3.0 \
  --search-speed 0.15 \
  --max-search-time 90.0 \
  --center-speed 0.2 \
  --servo-pin 12 \
  --servo-closed -0.5 \
  --servo-open 0.5
```

---

## 8. ArduPilot Parameter-Übersicht (MicoConfigurator)

Für den Betrieb ohne RC-Fernsteuerung und mit zwei LiDARs (Boden + Wand) müssen folgende Parameter im Flight Controller gesetzt sein:

```text
# 1. Autonomer Betrieb ohne RC
ARMING_CHECK     = 0        # PreArm RC-Check deaktivieren
FS_THR_ENABLE    = 0        # Throttle Failsafe deaktivieren

# 2. Boden-Sensor MTF-01P (Höhe & Optical Flow)
SERIAL5_PROTOCOL = 1        # MAVLink
SERIAL5_BAUD     = 115
FLOW_TYPE        = 5        # MicoAir Flow
RNGFND1_TYPE     = 10       # MicoAir Lidar
RNGFND1_ORIENT   = 25       # Downward

# 3. Vorwärts-Sensor MT-15 (Wand-Kollisionsschutz auf SERIAL2)
SERIAL2_PROTOCOL = 9        # Rangefinder
SERIAL2_BAUD     = 115
RNGFND2_TYPE     = 10       # ToF Rangefinder
RNGFND2_ORIENT   = 0        # Forward
RNGFND2_MIN_CM   = 10
RNGFND2_MAX_CM   = 1500

# 4. ArduPilot Native Notbremse
AVOID_ENABLE     = 2        # Proximity Avoidance
AVOID_MARGIN     = 1.0      # 1 Meter Sicherheitsabstand
AVOID_BEHAVE     = 1        # Stop
```
