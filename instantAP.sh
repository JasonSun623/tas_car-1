#!/bin/bash
## Instant WLAN Access-Point
## elektronenblitz63 ubuntuusers.de 2012
## published under GPL v3
##
## Version 1.6.0 vom 31.Januar 2012
# Bridged-Modus möglich 
# kleinere Fehler beseitigt
# Ausgaben verbessert
#
## Version 1.5.2 vom 12.November 2011
# voreingestellte freie DNS geändert
# Restart des Network-Managers erst bei -stop
# entferne iptables-Filter bei -stop
##
## Beispielkonfiguration der dnsmasq.conf
##
# # DHCP-Server dnsmasq aktiv für Interface
#
# interface=wlan0

## DHCP-Server dnsmasq nicht aktiv für Interface
#
# no-dhcp-interface=eth0

# # IP-Adressbereich / Lease-Time
# dhcp-range=192.168.3.20,192.168.3.25,infinite
#
## Ende Beispielkonfiguration dnsmasq.conf

## freie Variablen

## Konfiguration der Ethernet-Schnittstelle 
## LAN statisch (Standard ist automatisch über DHCP) / Startoption [-f]
laniface=eth0
laddress=192.168.178.6
lbroadcast=192.168.178.255
lnetmask=255.255.255.0
lgateway=192.168.178.1
lmacaddress=00:90:f5:f8:62:5b
#
## Konfiguration der WLAN-Schnittstelle
## WLAN statisch
wlaniface=wlan0
waddress=192.168.3.1
wbroadcast=192.168.3.255
wnetmask=255.255.255.0
iptablemask=192.168.0.0/24

### manuelle DNS (drei DNS Einträge, 1xDomain und 1xSearch sind möglich)
# Beispiel
# dns="nameserver 192.168.178.1 nameserver 192.168.178.1 nameserver 192.168.178.1 domain fritz.box search fritz.box"
dns="nameserver 8.8.4.4 nameserver 8.8.8.8 nameserver 213.73.91.35" 

## dnsmasq-base Konfiguration
# DHCP-Adresspool umfasst x-Adressen
ipaddresses=10

# Basisadresse DHCP-Adresspool (WLAN-IP + X)
wlanbaseip=1

# Lease-Time
leasetime=infinite

## MAC-Adresse (optional) (Startoption [-m])
lmacaddress=00:90:f5:f8:62:5b

## Pause vor LAN-Verbindungstest
pause=4

## Proxyserver (squid)
proxy="squid"

## Proxy Server auf Port x (squid 3128 / tinyproxy 3128 / polipo 8123)
proxyport=3128

## Bridge-Konfiguration
## vor Ubuntu 11.x - /usr/sbin/brctl
## ab Ubuntu 11.x - /sbin/brctl
br_util=/sbin/brctl
bridge0=br0
brdelay=5
brstp=0

## Steuerung Dienste
## alt
# hostapdrestart="/etc/init.d/hostapd"
## neu
hostapdservice="service hostapd"

## Ende freie Variablen
##
# Skript
#
pingout=""

## aut. Adressberechnung DHCP-Range für dnsmasq
## gemäß Vorgabe WLAN-Schnittstelle
ipaddresses=$[$ipaddresses+$wlanbaseip]
 baseendaddr="`echo $waddress | tr -s . " " | awk {'print $4'}`"
  basestartaddr="`echo $waddress | tr -s . " " | awk {'print $1,$2,$3'} | tr -s " " .`"
   endaddr="$basestartaddr""."$[$startaddr+$ipaddresses]
    startaddr="$basestartaddr""."$[$baseendaddr+$wlanbaseip]

pingout=""
A=1
B=0
D=0
P=0
Br=0

if [ "$1" = "-h" ]; then
echo Verwendung: instant_AP.sh [-start] [-restart] [-stop] [-D] [-d] [-f] [-m] [-h] [-B]
echo Syntax:
echo "sudo ./instant_AP.sh  wie [-d] startet mit Standardparametern (DHCP)"
echo "sudo ./instant_AP.sh -f statische LAN-Konfiguration"
echo "sudo ./instant_AP.sh -f -m  statische LAN-Konfiguration, MAC-Änderung"
echo "sudo ./instant_AP.sh -D verwendet dnsmasq.conf und nicht dnsmasq-base"
echo "sudo ./instant_AP.sh -start -f statische LAN-Konfiguration"
echo "sudo ./instant_AP.sh -restart -f statische LAN-Konfiguration"
echo "sudo ./instant_AP.sh  -B Bridged-Modus ohne DHCP-Server. Dynamische und Statische Schnittstellenparameter werden ignoriert"
echo "sudo ./instant_AP.sh -stop beendet den AP"
echo "Ende"
 exit
fi

while getopts ":DdfmhPB" OPTION ; do
 case $OPTION in
  D) echo "vewende dnsmasq.conf und nicht dnsmasq-base"; D=1;;
  d) echo "konfiguriere LAN über DHCP"; A=1;;
  f) echo "konfiguriere LAN statisch"; A=2;;
  m) echo "MAC-Change LAN ein"; B=1;;
  P) echo "Portumleitung für Proxy-Server Port" $proxyport "aktiviert"; P=1;;
  B) echo "Bridge-Modus aktiviert"; Br=1;;
 esac
  done

echo "starte gewählte Konfiguration ..."
 sleep 2

if [ "$Br" = "1" ]; then A=3
 echo "Bridge-Mode aktiviert - Sonstige Parameter für Schnittstelleneinstellungen werden ignoriert"
  fi

if [ "$1" != "-start" ]; then
 echo "stoppe alle Dienste, und Verbindungen, lösche Itables-Filter ..."

# Konfiguration löschen
/sbin/iptables -F
 /sbin/iptables -X
  /sbin/iptables -t nat -F

defgw="`route -n | grep UG | awk {'print $2'}`"
 /sbin/route del default gw $defgw $laniface
   echo '' | tee /etc/resolv.conf

$hostapdservice stop 
 /sbin/ifconfig $wlaniface down
  sleep 1
  /sbin/iwconfig $wlaniface mode managed
   sleep 1
   /sbin/ifconfig $laniface down
    sleep 1
     /usr/bin/killall dnsmasq
      /sbin/sysctl -w net.ipv4.ip_forward=0
       /sbin/modprobe -rfv iptable_nat ipt_MASQUERADE xt_conntrack iptable_filter   

## Bridge löschen
/sbin/ifconfig $bridge0 down
 sleep 1
  $br_util delif $bridge0 $laniface
   $br_util delif $bridge0 $wlaniface
    $br_util delbr $bridge0

if [ "$1" = "-stop" ]; then
 echo
  echo "reaktiviere Network-Manager."
   service network-manager start
    service network-manager restart
echo "WLAN Access-Point Konfiguration beendet."
 exit
  fi
 fi

## MAC-Adresse abgleichen
if [ "$B" = "1" ]; then
 currentmac="`ifconfig $laniface | grep Adresse | awk {'print $6'}`"
  echo Schnittstelle $laniface, MAC-Adresse: $currentmac 
   echo Vorgabe: $lmacaddress

 if [ "$currentmac" = "$lmacaddress" ]; then
   echo Übereinstimmende MAC-Adresse 
else
   /sbin/ifconfig $laniface down
 /sbin/ip link set dev $laniface addr $lmacaddress

currentmac="`ifconfig $laniface | grep Adresse | awk {'print $6'}`"
 echo versuche MAC-Adresse zu ändern ...
  echo Schnittstelle $laniface, MAC-Adresse: $currentmac 

if [ "$currentmac" = "$lmacaddress" ]; then
 echo Änderung der MAC-Adresse erfolgreich! 
  else
 echo Änderung der MAC-Adresse nicht erfolgreich!
echo fahre fort ...
fi
 fi
  fi

# Grundkonfiguration 
echo beende Network-Manager
 service network-manager stop
  echo "starte alle Dienste, und Verbindungen ..."

# LAN aut.m über DHCP nur wenn kein Bridged-Mode gewählt
if [ "$Br" = "0" ] & [ "$A" = "1" ]; then
 echo "starte automatische LAN-Verbindung ..."
  /sbin/dhclient $laniface
fi

# LAN statisch nur wenn kein Bridged-Mode gewählt
if [ "$Br" = "0" ] & [ "$A" = "2" ]; then
  echo "starte statische LAN-konfiguration ..."
   /sbin/ifconfig $laniface down
    sleep 2
     /sbin/ifconfig $laniface $laddress broadcast $lbroadcast netmask $lnetmask
      sleep 2
       echo

echo setze Gateway und Route ...
 /sbin/route add default gw $lgateway $laniface
  sleep 1
   echo

echo setze DNS
echo '# erzeugt durch instant_AdHoc.sh' | tee /etc/resolv.conf
 echo $dns | awk {'print $1,$2'} | tee -a /etc/resolv.conf
  echo $dns | awk {'print $3,$4'} | tee -a /etc/resolv.conf
   echo $dns | awk {'print $5,$6'} | tee -a /etc/resolv.conf 
    echo $dns | awk {'print $7,$8'} | tee -a /etc/resolv.conf
     echo $dns | awk {'print $9,$10'} | tee -a /etc/resolv.conf
 fi
 sleep $pause

## Vorbereitung Bridge
if [ "$Br" = "1" ]; then

## vorhandene Schnittstellenkonfiguration löschen
/sbin/ifconfig $wlaniface down
 sleep 1
  /sbin/ifconfig $wlaniface up
   sleep 1
    /sbin/ifconfig $wlaniface 0.0.0.0
     sleep 1
     /sbin/ifconfig $laniface down
      sleep 1
       /sbin/ifconfig $laniface up
        sleep 1

## Bridge anlegen
 $br_util addbr $bridge0
  $br_util addif $bridge0 $laniface
   $br_util stp $bridge0 $brstp
    $br_util setfd $bridge0 $brdelay
     sleep 2

## Dienste steuern
service dnsmasq stop
 echo "fordere IP-Adresse für Bridge an ..."
  /sbin/dhclient $bridge0
   $hostapdservice restart
    $br_util addif $bridge0 $wlaniface
else

## ohne Bridge-Mode
## WLAN-Schnittstelle statisch konfigurieren
echo "WLAN-Schnittstelle initialisieren ..."
 /sbin/ifconfig $laniface up
  sleep 1
   /sbin/ifconfig $wlaniface $waddress broadcast $wbroadcast netmask $wnetmask
    $hostapdservice restart
     echo
echo "starte IP-Forward, Masquerading und NAT"
/sbin/iptables -A FORWARD -o $laniface -i $wlaniface -s $iptablemask -m conntrack --ctstate NEW -j ACCEPT
 /sbin/iptables -A FORWARD -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  /sbin/iptables -t nat -A POSTROUTING -o $laniface -j MASQUERADE 
   /sbin/sysctl -w net.ipv4.ip_forward=1 
fi

## dnsmasq-base starten
echo
 echo starte dnsmasq-base
if [ "$D" = "0" ]; then
 echo DHCP-Range dnsmasq-base - Startadresse: $startaddr Endadresse: $endaddr
  /usr/sbin/dnsmasq -i $wlaniface -I $laniface -F $startaddr,$endaddr,$leasetime
   echo
else
 
## dnsmasq neu starten
echo "verwende dnsmasq.conf"
 /etc/init.d/dnsmasq restart
  echo fertig ...
fi

## Portumleitung für Squid Proxyserver
if [ "$P" = "1" ]; then
 /sbin/iptables -t nat -A PREROUTING -i $wlaniface -p tcp --dport 80 -j REDIRECT --to-port $proxyport

## optional Port 443 HTTPS
#  /sbin/iptables -t nat -A PREROUTING -i $wlaniface -p tcp --dport 443 -j REDIRECT --to-port $proxyport

    echo "Port 80 (HTTP) " $wlaniface "auf Port $proxyport umgeleitet ("$proxy "Proxyserver)"
   echo "Starte" $proxy "Proxyserver ..."
 sleep 2

if [ "$proxy" != "squid" ]; then
 /etc/init.d/$proxy restart
  else
   service squid start -n
 fi
fi

## Ausgabe der aktuellen Konfiguration
 echo "DNS-Konfiguration"
  cat /etc/resolv.conf
   echo
    /sbin/route -n
     echo

if [ "$Br" = "1" ]; then
 echo "Konfiguration Bridge:"
  $br_util show
   echo
    /sbin/ifconfig $bridge0 | egrep 'Link|inet Adresse'
     echo
fi
 echo "Konfiguration LAN:"
  /sbin/ifconfig $laniface | egrep 'Link|inet Adresse'
   echo
    echo "Konfiguration WLAN:"
     /sbin/ifconfig $wlaniface | egrep 'Link|inet Adresse'
      echo
       /sbin/iwconfig $wlaniface | egrep 'IEEE|Power|Mode'
        echo      
         /sbin/iwconfig mon.$wlaniface

exit 0
