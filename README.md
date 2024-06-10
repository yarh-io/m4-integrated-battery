# Overview

This is a battery monitor for [YARH.IO Micro 4](https://yarh.io/). The battery monitor consists of  [Adafruit LC709203F LiPoly / LiIon Fuel Gauge and Battery Monitor](https://www.adafruit.com/product/4712), a Linux kernel module and supporting OS service for Raspberry Pi2, 3, 4.

The kernel module creates a virtual battery in Linux that is reported like a laptop battery on the taskbar, etc. The service queries the Adafruit LC709203F over i2c and provides the battery status to the kernel module.

<img src="/images/2024-06-10-181315_800x480_scrot.png" alt="terminal screen"/>


# Wiring and software setup

    sudo apt install build-essential

    sudo apt install make raspberrypi-kernel-headers


## Build the module and test it

Clone the repo and make the module:

    $ cd m4-integrated-battery
    $ make

You should now see integrated_battery.ko listed.


## Install the service and test it

    $ sudo ln -s /home/pi/bin/m4-integrated-battery/battery_update_linux.service /etc/systemd/system/battery_update_linux.service

    $ sudo systemctl start battery_update_linux.service
    $ sudo systemctl status battery_update_linux.service
    $ sudo systemctl enable battery_update_linux.service


