#!/bin/bash
# Let normal users access the Hiwonder servo controller (USB 0483:5750).
set -e
cat > /etc/udev/rules.d/99-hiwonder-servo.rules <<"R"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE="0666"
R
udevadm control --reload-rules
udevadm trigger
echo "Servo controller access set up."
