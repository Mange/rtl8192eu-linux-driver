#! /usr/bin/env bash

for c in "sudo" "mktemp" "git" "dkms"; do
    command -v $c >/dev/null 2>&1 || { echo >&2 "Script uses $c, but it's not installed. Aborting."; exit 1; }
done

sudo echo # pre-authorise sudo

cd $(mktemp -d)

git clone --depth 1 https://github.com/Mange/rtl8192eu-linux-driver.git
cd rtl8192eu-linux-driver

sudo dkms remove rtl8192eu/1.0 --all
sudo dkms add .
sudo dkms install rtl8192eu/1.0
sudo modprobe 8192eu

cd ..
rm -rf $(pwd)

echo "blacklist wl" | sudo tee /etc/modprobe.d/blacklist-wl.conf
echo "blacklist rtl8xxxu" | sudo tee /etc/modprobe.d/blacklist-rtl8xxxu.conf
