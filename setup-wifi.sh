#!/bin/bash

sudo sh -c 'echo "network={" > /etc/wpa_supplicant/wpa_supplicant.conf'
sudo sh -c 'echo "  ssid=\"turtle19\"" >> /etc/wpa_supplicant/wpa_supplicant.conf'
sudo sh -c 'echo "  psk=\"Robotics&7\"" >> /etc/wpa_supplicant/wpa_supplicant.conf'
sudo sh -c 'echo "}" >> /etc/wpa_supplicant/wpa_supplicant.conf'
echo "/etc/wpa_supplicant/wpa_supplicant.conf"

if grep -q "auto wlx000f004e07e7" /etc/network/interfaces; then
echo "/etc/network/interfaces is configured"
else
sudo sh -c 'echo "auto wlx000f004e07e7" >> /etc/network/interfaces'
sudo sh -c 'echo "iface wlx000f004e07e7 inet dhcp" >> /etc/network/interfaces'
sudo sh -c 'echo "wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf" >> /etc/network/interfaces '
echo "/etc/network/interfaces is configured"
fi
