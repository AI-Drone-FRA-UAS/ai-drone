# Projektplakat

`plakat.html` ist das Poster im Format **A1 hoch (594 × 841 mm)**.
`plakat.pdf` ist der daraus erzeugte Druckstand.

## Fotos einsetzen

Es sind zwei Platzhalter vorgesehen (gestrichelter Rahmen, „Foto 1" / „Foto 2").
Bild nach `fotos/` legen und im HTML den Platzhalter-Inhalt durch ein `img` ersetzen:

```html
<div class="foto foto-gross">
  <img src="fotos/foto1.jpg" alt="Drohne mit Rahmenerweiterung">
</div>
```

Der gestrichelte Rahmen verschwindet automatisch, sobald ein `img` im Kasten steht.
Das Bild wird formatfüllend zugeschnitten (`object-fit: cover`), Querformat passt
für Foto 1, für Foto 2 eher ein etwas breiteres Motiv.

Für den Druck sollten die Fotos mindestens **1600 px** in der Breite haben.

## Eingesetzte Fotos

`fotos/drohne-flug-quer.jpg` (1800 × 1125) steckt als **Abb. 1** im Kasten „Die Drohne".
`fotos/drohne-flug-hoch.jpg` ist derselbe Flug im Hochformat und wird auf der
Projektseite (`site/`) als Titelbild verwendet. Beide sind aus dem Original
`Bild.png` (3840 × 2160, um 90° gedreht) erzeugt; das Original bleibt lokal und
ist per `.gitignore` ausgenommen, weil es 29 MB groß ist.

## Noch auszufüllen

- **Foto 2** (rechte Spalte, „Autonomer Testflug"): Testflug im Sicherheitsnetz
  oder ein Screenshot der Personenerkennung. Danach die Bildunterschrift
  **Abb. 3** ergänzen.
- **Autonomer Flug** (rechte Spalte, orange gestrichelt): Ergebnisse der Flugversuche
  und Erkenntnisse/Grenzen. Der Ansatz ist bereits beschrieben, die Linien darunter
  sind bewusst frei gelassen.

## PDF neu erzeugen

```bash
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu --no-pdf-header-footer --print-to-pdf=docs/poster/plakat.pdf "file://$PWD/docs/poster/plakat.html"
```

Alternativ im Browser öffnen und drucken: Papierformat **A1**, Ränder **keine**,
Option **Hintergrundgrafiken** aktivieren, Skalierung 100 %.

## Quellen des Inhalts

Der Text stammt aus `docs/drone-project.md`, `docs/DRONE_CONFIGURATION.md`,
`servo_instruction.md` und der `README.md`.
