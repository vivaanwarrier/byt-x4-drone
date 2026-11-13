#!/bin/sh
# Copy the code to the board and run main.py with output in this terminal.
# main.py is run from this computer, so it is not stored as the boot program.
# Stop with Ctrl-C.
set -e
cd "$(dirname "$0")/.."

MP=$(command -v mpremote || echo "$HOME/.local/bin/mpremote")

exec "$MP" connect auto fs cp config.py mpu6050.py motors.py receiver.py safety.py : + run main.py
