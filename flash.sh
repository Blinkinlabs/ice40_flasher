#!/bin/sh
set -e
cd "$(dirname "$0")"
. .venv/bin/activate
python reset_to_bootloader.py
sleep 1
picotool load build/main.uf2
picotool reboot
