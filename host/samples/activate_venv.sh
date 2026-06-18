#!/usr/bin/env bash
# Helper script to activate the Python virtual environment.
#
# Usage (must source, not execute):
#   source ./samples/activate_venv.sh
#
# This will activate the .venv virtual environment for the current shell.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

if [[ ! -d "$VENV_DIR" ]]; then
    echo "ERROR: Virtual environment not found at $VENV_DIR"
    echo "Initialize with: bash $SCRIPT_DIR/setup_venv.sh"
    return 1
fi

source "$VENV_DIR/bin/activate"
echo "Virtual environment activated. Python: $(which python3)"
