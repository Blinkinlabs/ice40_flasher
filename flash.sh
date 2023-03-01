#!/bin/sh
set -e

. .venv/bin/activate
python reset_to_bootloader.py
sleep 1
picotool load build/main.uf2
picotool reboot
