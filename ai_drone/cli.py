"""CLI entry point for the AI Drone project."""

from __future__ import annotations

import time

import typer
from pydantic import BaseModel
from rich import print as rprint

app = typer.Typer(help="AI Drone — camera, CV, and flight tools.")


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------


class DroneStatus(BaseModel):
    name: str
    battery: int
    armed: bool = False


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------


@app.command()
def status(
    name: str = typer.Option("Prototyp", help="Drone name"),
    battery: int = typer.Option(69, help="Battery level (%)"),
) -> None:
    """Print basic drone status (smoke test)."""
    s = DroneStatus(name=name, battery=battery)
    rprint(f"[bold green]{s.name}[/] battery: {s.battery}% | armed: {s.armed}")


@app.command()
def camera(
    frames: int = typer.Option(0, help="Capture N frames then exit (0 = live preview)"),
    output: str = typer.Option(
        "", help="Save last frame to this path instead of showing"
    ),
) -> None:
    """Open the camera and show a live preview or capture frames.

    On a Raspberry Pi this uses picamera2 + the IMX500 AI Camera.
    On a laptop it uses OpenCV (or a synthetic test pattern if no webcam).
    """
    from ai_drone.camera import Camera

    with Camera() as cam:
        rprint(f"[bold cyan]Camera backend:[/] {cam.backend}")

        if cam.backend in ("opencv", "synthetic"):
            _camera_loop_opencv(cam, frames, output)
        else:
            _camera_loop_headless(cam, frames, output)


@app.command()
def stream(
    port: int = typer.Option(8080, help="HTTP port to serve on"),
) -> None:
    """Stream live video from the camera over HTTP (MJPEG).

    Open http://<pi-ip>:<port>/ in a browser on your laptop.
    Press Ctrl-C to stop.
    """
    from ai_drone.camera import Camera
    from ai_drone.stream import run_stream

    with Camera() as cam:
        rprint(f"[bold cyan]Camera backend:[/] {cam.backend}")
        run_stream(cam, port=port)


@app.command()
def nearest(
    output: str = typer.Option(
        "stream", help="Output mode: stream | headless | display"
    ),
    regions: str = typer.Option(
        "", help="Path to 3D-calibration JSON (default: bundled regions)"
    ),
    confidence: float = typer.Option(0.40, help="Min detection confidence"),
    threshold: float = typer.Option(
        2.0, help="Distance (m) below which a pair is flagged"
    ),
    port: int = typer.Option(8080, help="HTTP port for stream mode"),
    rotate: int = typer.Option(0, help="Rotate output: 0, 90, 180 or 270 degrees"),
    altitude: float = typer.Option(
        0.0, help="Drone altitude (m); >0 derives distance from flight geometry"
    ),
    fov: float = typer.Option(
        66.0, help="Lens horizontal field of view (deg), for altitude mode"
    ),
) -> None:
    """Detect & track people and report the nearest neighbour distance.

    Runs the NanoDet model on the IMX500 AI Camera (Pi only, via modlib).
    Adapted from Sony's aitrios-rpi-sample-apps 'nearest-person' example.

    - stream  : annotated MJPEG at http://<pi-ip>:<port>/ (best for the drone)
    - headless: no image, logs nearest-pair distance per frame
    - display : OpenCV window (needs a desktop session)

    Distance calibration: a fixed-camera JSON (--regions) by default, or pass
    --altitude (with --fov) to compute metres-per-pixel live from drone height.
    """
    from ai_drone.nearest_person import run_nearest_person

    run_nearest_person(
        regions_file=regions or None,
        confidence=confidence,
        output=output,
        port=port,
        distance_threshold=threshold,
        rotate=rotate,
        altitude=altitude,
        fov=fov,
    )


# ---------------------------------------------------------------------------
# Camera loops
# ---------------------------------------------------------------------------


def _camera_loop_opencv(
    cam: object,
    frames: int,
    output: str,
) -> None:
    """Live preview using OpenCV highgui (laptop)."""
    import cv2  # type: ignore[import-untyped]
    from ai_drone.camera import Camera

    assert isinstance(cam, Camera)
    count = 0

    while True:
        frame = cam.capture()
        cv2.imshow("AI Drone — Camera", frame)
        count += 1

        if frames > 0 and count >= frames:
            break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    if output:
        cv2.imwrite(output, frame)  # noqa: F821 — frame always assigned
        rprint(f"[green]Saved frame to {output}[/]")

    cv2.destroyAllWindows()


def _camera_loop_headless(
    cam: object,
    frames: int,
    output: str,
) -> None:
    """Headless capture loop (Pi — no display attached)."""
    import cv2  # type: ignore[import-untyped]
    from ai_drone.camera import Camera

    assert isinstance(cam, Camera)

    n = frames if frames > 0 else 10
    rprint(f"[dim]Capturing {n} frame(s) headless…[/]")

    frame = None
    for i in range(n):
        frame = cam.capture()
        h, w = frame.shape[:2]
        rprint(f"  frame {i + 1}/{n}  {w}×{h}")
        if frames == 0:
            time.sleep(0.5)

    if output and frame is not None:
        cv2.imwrite(output, frame)
        rprint(f"[green]Saved last frame to {output}[/]")
    elif frame is not None:
        fallback = "/tmp/ai_drone_capture.jpg"
        cv2.imwrite(fallback, frame)
        rprint(f"[green]Saved last frame to {fallback}[/]")
