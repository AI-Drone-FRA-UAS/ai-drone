"""Small command adapters; flight and actuator lifetimes remain explicit."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from typing import Any

from ai_drone.settings import Settings, use_settings


def invoke(
    command: Callable[[Sequence[str]], int],
    arguments: Sequence[str],
    settings: Settings,
) -> int:
    with use_settings(settings):
        return command(arguments)


@contextmanager
def connection_scope(
    connection: Any, *, close_error: Callable[[Exception], None] | None = None
) -> Iterator[Any]:
    """Share close mechanics while callers retain their selected connection policy."""
    try:
        yield connection
    finally:
        try:
            connection.close()
        except Exception as error:
            if close_error is None:
                raise
            close_error(error)
