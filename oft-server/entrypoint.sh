#!/usr/bin/env bash
set -euo pipefail

MODE="${OFT_MODE:-mock}"
PORT="${OFT_PORT:-7071}"

if [ "$MODE" = "real" ] && [ -d /opt/openvla-oft ]; then
    pip install -e /opt/openvla-oft >/dev/null
fi

if [ "$MODE" = "mock" ]; then
    exec uvicorn mock_server:app --host 0.0.0.0 --port "$PORT"
else
    exec uvicorn server:app --host 0.0.0.0 --port "$PORT"
fi
