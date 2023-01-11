#!/bin/bash

echo "install ..."

# Update apt sources
sudo cp /etc/apt/sources.list /etc/apt/sources.list.org
sudo sed -i -e 's/ports.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list

sudo apt-get update

# Set user group
sudo usermod -a -G dialout $USER

# Installation
sudo apt install gpiod libgpiod-dev
sudo apt install gpsd gpsd-clients pps-tools linuxptp chrony
sudo apt install libpcap-dev libyaml-cpp-dev

# Custom
if [ -e MVS-2.1.2_aarch64_20221024.deb ]; then
    sudo dpkg -i MVS-2.1.2_aarch64_20221024.deb
fi

if [ -e networkmanager-conf-wifi.tgz ]; then
    tar zxvf networkmanager-conf-wifi.tgz -C /
fi

echo "done!"