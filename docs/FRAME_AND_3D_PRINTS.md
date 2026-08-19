# Frame Extension and 3D Prints

Goal 1.4 of the [Implementation Reference](drone-project.md) is a frame
extension that carries the MTF-01P, the payload servo, and the Raspberry Pi
with its camera. This document lists the print files that exist in the
repository, what each one is, and how they were sliced.

---

## 1. What is in `3DPrints/`

```text
3DPrints/
├── Body-withSupports.stl      Raspberry Pi Zero enclosure body
├── Body-withSupports.3mf      the same part, sliced for a Prusa XL
└── drone-rubber-prusa/
    ├── front_rubber_clean_oriented_prusa.{stl,3mf}
    ├── rear_rubber_clean_oriented_prusa.{stl,3mf}
    └── side_rubber_clean_oriented_prusa.{stl,3mf}
```

Each part is stored twice on purpose. The **STL** is the geometry — open it in
any slicer, on any printer. The **3MF** carries the part *and* the settings it
was actually printed with, so an identical print needs no re-configuration.
Edit geometry from the STL; reprint from the 3MF.

| Part | Size (X × Y × Z) | Material | What it is |
|------|------------------|----------|------------|
| `Body-withSupports` | 37.6 × 72.6 × 13.4 mm | PETG | Enclosure body for the Raspberry Pi Zero 2 WH. Supports are part of the model, not slicer-generated |
| `front_rubber` | 32.5 × 18.3 × 35.0 mm | TPU 95A | Front soft frame part of the BEE35 duct assembly |
| `rear_rubber` | 26.2 × 33.7 × 34.0 mm | TPU 95A | Rear soft frame part |
| `side_rubber` | 20.8 × 11.5 × 35.0 mm | TPU 95A | Side soft frame part — two are needed, one per side |

The soft parts are printed in flexible filament because they double as
vibration isolation. Optical flow and the EKF3 state estimate are sensitive to
frame vibration, so printing these in a rigid filament is not a substitution —
it changes how well the drone holds position indoors.

---

## 2. Print settings

Taken from the 3MF files, which were sliced in PrusaSlicer for an
**Original Prusa XL, 2 tool heads, 0.4 mm nozzle**, profile
`0.20mm SPEED @XL 0.4`.

| Setting | Body | Soft parts |
|---------|------|------------|
| Layer height | 0.20 mm | 0.20 mm |
| First layer | 0.20 mm | 0.20 mm |
| Perimeters | 2 | 2 |
| Infill | 30 % | 30 % |
| Filament | PETG | Prusament TPU 95A + PETG |
| Nozzle temperature | 240 °C | 225 °C (TPU) / 240 °C (PETG) |
| Bed temperature | 80 °C | 65 °C (TPU) / 80 °C (PETG) |
| Slicer supports | off | off |

The soft parts are two-material prints with a wipe tower. On a single-extruder
printer, load TPU 95A and print them in one material — the PETG in the profile
is the second tool, not a structural requirement of the part.

`Body-withSupports` has its supports modelled into the geometry, which is why
slicer support generation is off. Do not turn it on; you will get supports on
top of supports.

---

## 3. Slicing and printing

With PrusaSlicer or another slicer that reads 3MF:

```text
File → Import → Import 3MF, then slice and export the G-code.
```

With Cura, or any other slicer:

```text
Import the .stl, then apply the settings from the table above.
```

The parts are already oriented for printing — `*_oriented_prusa` in the file
name means the rotation is baked in. Re-orienting a flexible part usually
makes it worse, not better.

---

## 4. What the frame extension still has to carry

The pieces above are the enclosure and the soft frame parts. The mounting
plate that ties the whole payload together is still open work. It has to hold:

| Component | Requirement |
|-----------|-------------|
| MicoAir MTF-01P | Pointing straight down, with nothing in its field of view — that includes the payload and the payload arm |
| Payload servo + release arm | Rigid enough that the arm does not flex out of the retained position — see [Payload Drop Mechanism](PAYLOAD_DROP.md) |
| Raspberry Pi Zero 2 WH | Uses the `Body` enclosure; the UART4 cable and the USB port must stay reachable |
| Pi AI Camera (IMX500) | Forward-facing for person detection; the CSI ribbon has a tight bend radius, so leave slack |

Mounting hardware provided with the kit: M3 standoffs with M3×9 mm and
M3×12 mm screws.

Two constraints apply to anything added to this airframe:

- **Mass.** A 3.5" CineWhoop with ducts and propeller guards has limited
  headroom. Every gram added to the extension comes off the flight time, and
  the drop mechanism has to lift the payload as well.
- **Balance.** The payload hangs below the centre of gravity. Mount it on the
  centreline; an offset load shows up as a constant attitude correction and
  degrades position hold.

CAD is done in Tinkercad and sliced in Cura or PrusaSlicer, per the
[Implementation Reference](drone-project.md).
