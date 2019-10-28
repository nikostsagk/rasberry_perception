#!/usr/bin/env bash
set -e
cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)

# Install custom USB CAN rules
sudo cp rules/79-usb2can-rw.rules /etc/udev/rules.d/79-usb2can.rules
sudo cp rules/79-usb2gripper.rules /etc/udev/rules.d/79-usb2gripper.rules
sudo udevadm control --reload
sudo service udev restart
