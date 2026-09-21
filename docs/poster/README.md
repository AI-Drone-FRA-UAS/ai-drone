# Projektplakat

`plakat.html` ist die Quelle: eine einzelne HTML-Datei, gesetzt für **A1 hoch
(594 × 841 mm)**, ohne externe Abhängigkeiten. Daraus entstehen vier Druckstände:

| Datei | Format | Wofür |
|---|---|---|
| `plakat-a0.pdf` | 841 × 1189 mm | Messestand, Lesen aus mehreren Metern |
| `plakat-a1.pdf` | 594 × 841 mm | das Zielformat, für das die Schriftgrößen gewählt sind |
| `plakat-a2.pdf` | 420 × 594 mm | Bürowand, kleine Posterwand |
| `plakat-a3.pdf` | 297 × 420 mm | Handout und Korrekturausdruck |

Nur A1 wird gerendert; A0, A2 und A3 sind maßstäbliche Skalierungen derselben
Seite. Schrift und Grafik bleiben dabei vektoriell.

## Aufbau

Das Plakat ist bewusst bildlastig — der Text ist knapp gehalten, damit beim
Vortrag frei geredet werden kann.

- **Oben:** das Hauptbild der Drohne über zwei Spalten, mit Führungslinien zu
  den Bauteilen. Kamera (schwarze Halterung), MTF-01P (silbernes Gehäuse)
  und Servo sind einzeln zugeordnet. Die Beschriftung liegt als SVG über dem
  Foto, ist also vektoriell und bleibt im Druck scharf. Sie ersetzt die frühere Hardwaretabelle.
- **Links:** Kennzahlen und der Blick von unten mit nummerierten Markern
  und Führungslinien (Servo, MTF-01P, Kamera).
- **Mitte:** die Ablaufgrafik „Tag erkannt → Last fällt", das Foto des
  Abwurfmechanismus und die gedruckten Teile. Die Last ist im senkrechten Fall
  vor dem Auftreffen dargestellt, damit der AprilTag sichtbar bleibt.
- **Rechts:** die Geschichte von oben nach unten — erst der Simulator, dann die
  Flugversuche, der Absturz mit Messkurve und QR-Code zum Video, die beiden
  dargestellten Befunde, die Konsequenzen und der datierte Projektstand.

## Bilder

| Datei | Verwendung |
|---|---|
| `fotos/Bild neu.jpg` | Originalaufnahme: die Drohne hängt an einem Kabelbinder |
| `fotos/draufsicht-ohne-hand.jpg` | daraus das Hauptbild: Hand und Kabelbinder herausgerechnet, zugeschnitten |
| `fotos/Draufsicht.jpg` | ältere Aufnahme derselben Ansicht (Finger vor der Kamera) |
| `fotos/Servo 1.jpg` | Originalaufnahme des Abwurfmechanismus |
| `fotos/abwurf-servo.jpg` | daraus der Zuschnitt für Abb. 2 |
| `fotos/Bild von unten Lidar Servo Kamera.jpg` | Original der Unterseite |
| `fotos/blick-von-unten.jpg` | daraus der Zuschnitt mit den drei Bauteilen |
| `fotos/STL-Screenshot*.jpg` | unveränderte STL-Ansichten in dunklen Bildkacheln; alle Seitenflächen bleiben erhalten |
| `fotos/teil-*.png` | frühere freigestellte Ableitungen, nicht mehr im Plakat verwendet |
| `fotos/drohne-flug-*.jpg` | Flugaufnahmen, genutzt auf der Projektseite |

Hand und Kabelbinder im Hauptbild wurden nicht übermalt, sondern
herausgerechnet: Die Unterkante des Arms wird spaltenweise an der Hautfarbe
verfolgt, die beiden Binderstränge als schmale, farblose Bänder erfasst, die
Drohne selbst geschützt, und die Wandfläche per Mehrgitter-Diffusion
rekonstruiert. Damit die Füllung keine Farbe von Klett und Kabeln zieht, sind
alle nicht-wandigen Pixel in der Nachbarschaft von der Farbquelle
ausgenommen. Die abgeleiteten Dateien lassen sich aus den Originalen jederzeit
neu erzeugen.

Die STL-Ansichten verwenden die Original-Screenshots ohne Freistellmaske.
Bei den früheren PNG-Ableitungen wurden dunkle Seitenflächen teilweise als
Hintergrund entfernt. `object-fit: contain` zeigt die vollständigen Originale
ohne Beschnitt.

Der **QR-Code** ist als Pfad-SVG direkt im HTML eingebettet (kein externer
Dienst, kein Bild-Asset) und zeigt auf das Video des Absturzes vom 21.08.2026
auf Instagram: `instagram.com/reel/DcbRrKNRIA_`. Eine weiße Ruhezone von
vier Modulen umgibt den Code. Bewusst ohne Tracking- und
Share-Parameter — das hält den Code klein und hängt keinen Sitzungsbezug an
einen gedruckten Aushang.
Der **AprilTag** in der Ablaufgrafik ist der echte `tag36h11`-Marker mit ID 3 —
derselbe, auf den die Drohne reagiert.

## PDFs neu erzeugen

Zuerst A1 aus dem HTML rendern:

```bash
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu --no-pdf-header-footer --print-to-pdf=docs/poster/plakat-a1.pdf "file://$PWD/docs/poster/plakat.html"
```

Dann die drei übrigen Formate daraus skalieren — reine Skalierung derselben
Seite, damit alle vier Druckstände denselben Satz zeigen:

```bash
cd docs/poster && uv run --with pymupdf python skaliere_plakat.py
```

Der Inhalt von `skaliere_plakat.py` (bewusst nicht im Repository, damit keine
Abhängigkeit deklariert werden muss, die nur ein Druckvorgang braucht):

```python
import pymupdf

MM = 72 / 25.4
ZIELE = {"a0": (841.0, 1189.0), "a2": (420.0, 594.0), "a3": (297.0, 420.0)}

with pymupdf.open("plakat-a1.pdf") as quelle:
    for name, (breite, hoehe) in ZIELE.items():
        with pymupdf.open() as ziel:
            seite = ziel.new_page(width=breite * MM, height=hoehe * MM)
            # Die ganze Seite als Form-XObject einbetten: So werden auch
            # Farbverläufe und Transparenzen mit dem Inhalt skaliert.
            seite.show_pdf_page(seite.rect, quelle, 0, keep_proportion=True)
            ziel.save(f"plakat-{name}.pdf", garbage=4, deflate=True)
        print(f"plakat-{name}.pdf: {breite:.0f} x {hoehe:.0f} mm")
```

Alternativ im Browser öffnen und drucken: Papierformat wählen, Ränder **keine**,
Option **Hintergrundgrafiken** aktivieren, Skalierung „an Seite anpassen".

## Stand der Angaben

September 2026: eigene ArduCopter-4.7.1-Firmware mit EKF3-Flussfusion, vorderer
MT-15 an UART3 in Betrieb, implementierte
einmalige Freigabe der Halterung pro Programmlauf bei Tag ID 3 (ohne
automatisches Schließen). Die Kennzahlen unterscheiden die direkt gemessene
MT-15-Sensorrate von der Kameraauswertung im Sensortest. Der offene
Blocker ist die Kompass-Vorflugprüfung; die Drohne ist nicht für Tests mit
Propellern freigegeben. Wer das Plakat anfasst, prüft diese Angaben zuerst
gegen die neueste Aufzeichnung unter `state/`.

Quellen: `docs/drone-project.md`, `docs/DRONE_CONFIGURATION.md`,
[ArduCopter 4.7 no-GPS Loiter](../ARDUCOPTER_4_7_NOGPS_LOITER.md),
[Sensoraufzeichnung](../SENSOR_RECORDING.md), die Aufzeichnungen unter
`state/2026-09-09/` und das
[archivierte Flugprotokoll](../../notes/archive/preflight-and-nogps-takeoff/README.md).
