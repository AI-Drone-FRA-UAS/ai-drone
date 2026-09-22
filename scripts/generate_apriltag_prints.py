"""Generate tag36h11 print sheets for DIN A4 or A3.

From the repository root:
    uv run --locked --group raspi python scripts/generate_apriltag_prints.py \
        --ids 0-9 --paper A3 --out artifacts/apriltags --pdf

Each sheet centres an 8-cell black square on the page and surrounds it with a
one-cell white quiet zone. The cell pitch fixes the pose-estimation tag size:
20 mm cells give a 160 mm square on A4, 28 mm cells a 224 mm square on A3.

Sheets reproduce the AprilRobotics `apriltag-imgs` encodings byte for byte, so
output for IDs 0-4 is identical to the sheets already printed for the project.
PDF rendering needs `rsvg-convert` (Debian/Arch package `librsvg`).
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess

import cv2
import numpy

PAPER = {  # name: (page width mm, page height mm, cell pitch mm)
    "A4": (210.0, 297.0, 20.0),
    "A3": (297.0, 420.0, 28.0),
}
SQUARE_CELLS = 8  # six data bits plus the one-cell black border
GRID_CELLS = SQUARE_CELLS + 2  # plus the one-cell white quiet zone


def _mm(value: float) -> str:
    return f"{value:g}"


def tag_bits(tag_id: int) -> numpy.ndarray:
    """Return the 8x8 cell grid for ``tag_id``, 0 for black and 255 for white."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    marker = cv2.aruco.generateImageMarker(
        dictionary, tag_id, SQUARE_CELLS, borderBits=1
    )
    # apriltag-imgs renders tag36h11 rotated 180 degrees from OpenCV's ArUco
    # dictionary. Detectors decode either orientation to the same ID, but the
    # reported pose yaw differs, so keep every printed sheet on one convention.
    return numpy.rot90(marker, 2)


def svg_for(tag_id: int, paper: str) -> str:
    page_width, page_height, cell = PAPER[paper]
    block = GRID_CELLS * cell
    offset_x = (page_width - block) / 2.0
    offset_y = (page_height - block) / 2.0
    square_mm = round(SQUARE_CELLS * cell)

    bits = tag_bits(tag_id)
    rects = "".join(
        f'<rect x="{column + 1}" y="{row + 1}" width="1" height="1"/>'
        for row in range(SQUARE_CELLS)
        for column in range(SQUARE_CELLS)
        if bits[row][column] == 0
    )
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg"\n'
        f'     width="{_mm(page_width)}mm" height="{_mm(page_height)}mm"\n'
        f'     viewBox="0 0 {_mm(page_width)} {_mm(page_height)}">\n'
        f"  <title>AprilTag tag36h11 ID {tag_id} — {square_mm} mm "
        "pose-estimation square</title>\n"
        "  <desc>Print at 100 percent / Actual Size.</desc>\n"
        f'  <rect width="{_mm(page_width)}" height="{_mm(page_height)}" fill="#fff"/>\n'
        "  <g transform="
        f'"translate({_mm(offset_x)} {_mm(offset_y)}) scale({_mm(cell)})"\n'
        '     fill="#000" shape-rendering="crispEdges">\n'
        f"    {rects}\n"
        "  </g>\n"
        "</svg>\n"
    )


def parse_ids(text: str) -> range:
    first, _, last = text.partition("-")
    start, stop = int(first), int(last or first)
    if not 0 <= start <= stop <= 586:
        raise argparse.ArgumentTypeError("IDs must be an 0-586 range, low first")
    return range(start, stop + 1)


def render_pdf(sources: list[pathlib.Path], target: pathlib.Path) -> None:
    subprocess.run(
        ["rsvg-convert", "-f", "pdf", "-o", str(target), *map(str, sources)],
        check=True,
    )


def main(arguments: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", type=parse_ids, required=True, metavar="FIRST-LAST")
    parser.add_argument("--paper", choices=sorted(PAPER), default="A3")
    parser.add_argument("--out", type=pathlib.Path, required=True, metavar="DIRECTORY")
    parser.add_argument("--pdf", action="store_true", help="also render PDFs")
    args = parser.parse_args(arguments)

    square_mm = round(SQUARE_CELLS * PAPER[args.paper][2])
    args.out.mkdir(parents=True, exist_ok=True)
    written: list[pathlib.Path] = []

    for tag_id in args.ids:
        stem = f"tag36h11-id{tag_id}-{args.paper}-{square_mm}mm"
        svg = args.out / f"{stem}.svg"
        svg.write_text(svg_for(tag_id, args.paper), encoding="utf-8")
        written.append(svg)
        if args.pdf:
            render_pdf([svg], svg.with_suffix(".pdf"))
        print(stem)

    if args.pdf and len(written) > 1:
        first, last = args.ids[0], args.ids[-1]
        combined = args.out / (
            f"tag36h11-ids{first}-{last}-{args.paper}-{square_mm}mm"
            f"-{len(written)}pages.pdf"
        )
        render_pdf(written, combined)
        print(combined.name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
