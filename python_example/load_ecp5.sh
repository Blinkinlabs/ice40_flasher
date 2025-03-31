#!/usr/bin/bash
set -e

cd `dirname "$0"`

. .venv/bin/activate
python load_ecp5.py "$@"
