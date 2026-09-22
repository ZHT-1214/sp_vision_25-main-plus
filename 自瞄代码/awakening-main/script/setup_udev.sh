#!/usr/bin/env bash

sudo tee /etc/udev/rules.d/99-stm32-acm.rules > /dev/null <<EOF

SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="2069398C464D", SYMLINK+="stm32_acm"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
