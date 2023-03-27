#!/usr/bin/env bash

# Set up virtual environment
python3.10 -m venv .venv
source venv/bin/activate

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
PACKAGE_NAME="$(basename "$PACKAGE_DIR")"

# Install Pip dependencies
# (If this package has no additional Python dependencies, you should delete requirements.txt)
cd "$SCRIPT_DIR"
if [ -f "./requirements.txt" ]; then
    pip install -r requirements.txt
fi