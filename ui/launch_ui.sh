#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -n "${X3PLUS_PYTHON:-}" && -x "${X3PLUS_PYTHON}" ]]; then
  PY="${X3PLUS_PYTHON}"
elif [[ -x "$HOME/grasp_venv/bin/python3" ]]; then
  PY="$HOME/grasp_venv/bin/python3"
elif [[ -x "$ROOT/.venv/bin/python3" ]]; then
  PY="$ROOT/.venv/bin/python3"
else
  PY="$(command -v python3)"
fi

exec "$PY" "$ROOT/ui/launcher.py" --allow-real
