#!/usr/bin/env bash
set -euo pipefail

# Initialize Python virtual environment for sync analysis scripts.
# 
# Usage:
#   ./samples/setup_venv.sh
#
# This script:
#   1. Creates .venv with --system-site-packages (required for UHD bindings)
#   2. Installs dependencies from requirements.txt
#   3. Prints activation instructions

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

echo "========================================"
echo "Setting up Python Virtual Environment"
echo "========================================"
echo ""

if [[ -d "$VENV_DIR" ]]; then
    echo "✓ Virtual environment already exists at $VENV_DIR"
    echo ""
else
    echo "Creating virtual environment with --system-site-packages..."
    python3 -m venv --system-site-packages "$VENV_DIR"
    echo "✓ Virtual environment created"
    echo ""
fi

echo "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

echo "Installing dependencies from requirements.txt..."
pip install --upgrade pip setuptools wheel > /dev/null
pip install -r "$SCRIPT_DIR/requirements.txt"
echo "✓ Dependencies installed"
echo ""

echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "To activate the virtual environment in a new shell:"
echo "  source $SCRIPT_DIR/.venv/bin/activate"
echo ""
echo "Or use the included helper:"
echo "  source $SCRIPT_DIR/activate_venv.sh"
echo ""
echo "Then run scripts:"
echo "  ./capture_sync.py --serials S0 S1 [options]"
echo "  ./analyze_sync.py --files cap_S0.npy cap_S1.npy [options]"
echo ""
