#!/bin/bash

ID=`id -un`
TS=`date "+%Y-%m-%d-%H-%M-%S"`.pcap
mkdir -p ~/data
cd ~/data
sudo /usr/sbin/tcpdump -i $1 -Z $ID -s 0 -w $TS
