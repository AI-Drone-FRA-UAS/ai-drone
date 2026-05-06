# Project ai-drone for Fra-Uas with Professor Baun

## Setup

### 1. Install uv

**Linux / WSL:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Windows:**
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 2. Install dependencies

```bash
uv sync
```

### 3. Run

```bash
uv run main.py
```

With optional parameters:
```bash
uv run main.py --name prototyp-drone --battery 45
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--name`  | `Prototyp` | Drone name |
| `--battery` | `69` | Battery level (%) |

---

### other software

#### Ardu-pilot:
- stm32cubeprogrammer
