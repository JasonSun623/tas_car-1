hostapd  /etc/hostapd.conf &
/usr/sbin/dnsmasq -i wlan0  -F 192.168.3.10,192.168.3.100,360000 -d

