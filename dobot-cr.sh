#!/usr/bin/env bash
# Convenience wrapper script for dobot-cr CLI

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/venv/bin/activate"
exec dobot-cr "$@"
