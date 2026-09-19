# Contributing

Use short-lived branches from `main` and focused pull requests. Explain the
changed behavior and validation. Read [development and architecture](docs/SOFTWARE_ARCHITECTURE.md)
for checks, simulator setup, deployment and hardware ownership.

Use `uv` for Python. Keep hardware imports lazy and tests runnable without a Pi.
Keep functions focused; Ruff enforces a McCabe complexity limit of 10. Split
distinct responsibilities into named helpers instead of suppressing `C901`.
CI also runs `uv run --group dev ruff check . --select C901 --ignore-noqa`
so an inline suppression cannot hide a complex function.
Preserve dated captures in `state/`, parameter snapshots in `params/`, and raw
recordings in ignored `artifacts/`. Never substitute another aircraft's data.
