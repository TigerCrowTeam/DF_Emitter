#!/usr/bin/env bash
set -eu

DEMO_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
DEMO_PYTHON="$DEMO_ROOT/.venv/bin/python"
cd "$DEMO_ROOT"

if [ "$#" -eq 0 ]; then
    set -- --scenario dramatic --seed 42 --timeout 300 --keep-open
fi

exec "$DEMO_PYTHON" -m kraken_df_nav.mavproxy_orchestrator "$@"
