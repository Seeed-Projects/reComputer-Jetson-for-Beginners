#!/usr/bin/env bash
set -e

sudo apt update
sudo apt install -y python3-pip libzbar-dev
python3 -m pip install qrcode pyzbar
