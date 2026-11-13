#!/bin/sh
# Copy the code to the board so main.py runs on every power-up, then reboot it.
set -e
cd "$(dirname "$0")/.."

MP=$(command -v mpremote || echo "$HOME/.local/bin/mpremote")

"$MP" connect auto fs cp config.py mpu6050.py main.py : + reset
echo "Deployed. main.py now runs on boot."
echo "Watch it with:  mpremote connect auto repl   (Ctrl-] to leave)"
