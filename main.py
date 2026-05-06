import typer
from pydantic import BaseModel
from rich import print

app = typer.Typer()


class DroneStatus(BaseModel):
    name: str
    battery: int
    armed: bool = False


@app.command()
def main(name: str = "Prototyp", battery: int = 69) -> None:
    status = DroneStatus(name=name, battery=battery)
    print(
        f"[bold green]{status.name}[/] battery: {status.battery}% | armed: {status.armed}"
    )


if __name__ == "__main__":
    app()
