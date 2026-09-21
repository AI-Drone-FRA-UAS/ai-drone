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
  den Bauteilen. Die Beschriftung liegt als SVG über dem Foto, ist also
  vektoriell und bleibt im Druck scharf. Sie ersetzt die frühere Hardwaretabelle.
- **Links:** Kennzahlen und der Blick von unten mit nummerierten Markern
  (Servo, MTF-01P, Kamera).
- **Mitte:** die Ablaufgrafik „Tag erkannt → Last fällt", das Foto des
  Abwurfmechanismus und die gedruckten Teile.
- **Rechts:** die Geschichte von oben nach unten — erst der Simulator, dann die
  Flugversuche, der Absturz mit Messkurve und QR-Code zum Video, die drei
  gemessenen Ursachen, die Konsequenzen und der heutige Stand.

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
| `fotos/teil-*.png` | die STL-Ansichten, freigestellt und in die Plakatfarben gebracht |
| `fotos/drohne-flug-*.jpg` | Flugaufnahmen, genutzt auf der Projektseite |

Hand und Kabelbinder im Hauptbild wurden nicht übermalt, sondern
herausgerechnet: Die Unterkante des Arms wird spaltenweise an der Hautfarbe
verfolgt, die beiden Binderstränge als schmale, farblose Bänder erfasst, die
Drohne selbst geschützt, und die Wandfläche per Mehrgitter-Diffusion
rekonstruiert. Damit die Füllung keine Farbe von Klett und Kabeln zieht, sind
alle nicht-wandigen Pixel in der Nachbarschaft von der Farbquelle
ausgenommen. Die abgeleiteten Dateien lassen sich aus den Originalen jederzeit
neu erzeugen.

Der **QR-Code** ist als Pfad-SVG direkt im HTML eingebettet (kein externer
Dienst, kein Bild-Asset) und zeigt auf das Video des Absturzes vom 21.08.2026
auf Instagram: `instagram.com/reel/DcbRrKNRIA_`. Bewusst ohne Tracking- und
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
cd docs/poster && uv run --with pypdf python skaliere_plakat.py
```

Der Inhalt von `skaliere_plakat.py` (bewusst nicht im Repository, damit keine
Abhängigkeit deklariert werden muss, die nur ein Druckvorgang braucht):

```python
from pypdf import PdfReader, PdfWriter, Transformation
from pypdf.generic import RectangleObject

MM = 72 / 25.4
QUELLE = (594.0, 841.0)  # das Layout ist in A1 gesetzt
ZIELE = {"a0": (841.0, 1189.0), "a2": (420.0, 594.0), "a3": (297.0, 420.0)}

for name, (breite, hoehe) in ZIELE.items():
    seite = PdfReader("plakat-a1.pdf").pages[0]
    faktor = min(breite / QUELLE[0], hoehe / QUELLE[1])
    # Die DIN-Reihe rundet auf ganze Millimeter; den Rest mittig verteilen.
    dx = (breite - QUELLE[0] * faktor) / 2 * MM
    dy = (hoehe - QUELLE[1] * faktor) / 2 * MM
    seite.add_transformation(Transformation().scale(faktor).translate(dx, dy))
    kasten = RectangleObject((0, 0, breite * MM, hoehe * MM))
    seite.mediabox = kasten
    seite.cropbox = kasten
    schreiber = PdfWriter()
    schreiber.add_page(seite)
    schreiber.compress_identical_objects()
    for fertig in schreiber.pages:
        fertig.compress_content_streams(level=9)
    with open(f"plakat-{name}.pdf", "wb") as datei:
        schreiber.write(datei)
    print(f"plakat-{name}.pdf: {breite:.0f} x {hoehe:.0f} mm")
```

Alternativ im Browser öffnen und drucken: Papierformat wählen, Ränder **keine**,
Option **Hintergrundgrafiken** aktivieren, Skalierung „an Seite anpassen".

## Stand der Angaben

September 2026: eigene ArduCopter-4.7.1-Firmware mit EKF3-Flussfusion, vorderer
MT-15 an UART3 in Betrieb, Freigabe der Halterung bei Tag ID 3. Der offene
Blocker ist die Kompass-Vorflugprüfung; die Drohne ist nicht für Tests mit
Propellern freigegeben. Wer das Plakat anfasst, prüft diese Angaben zuerst
gegen die neueste Aufzeichnung unter `state/`.

Quellen: `docs/drone-project.md`, `docs/DRONE_CONFIGURATION.md`,
[ArduCopter 4.7 no-GPS Loiter](../ARDUCOPTER_4_7_NOGPS_LOITER.md),
[Sensoraufzeichnung](../SENSOR_RECORDING.md), die Aufzeichnungen unter
`state/2026-09-09/` und das
[archivierte Flugprotokoll](../../notes/archive/preflight-and-nogps-takeoff/README.md).
