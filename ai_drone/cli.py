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
    rprint(
        f"[bold green]{s.name}[/] battery: {s.battery}% | armed: {s.armed}"
    )


@app.command()
def camera(
    frames: int = typer.Option(0, help="Capture N frames then exit (0 = live preview)"),
    output: str = typer.Option("", help="Save last frame to this path instead of showing"),
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
