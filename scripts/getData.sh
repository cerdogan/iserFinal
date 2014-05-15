#!/bin/sh

killall achd
rm -rf /dev/shm/ach*

ach -o 666 -C imu-data || exit 1
ach -o 666 -C waist-state || exit 1
ach -o 666 -C amc-state || exit 1
ach -o 666 -C dynamixel-state || exit 1
ach -o 666 -C dynamixel-cmd || exit 1
ach -o 666 -C cinder || exit 1
ach -o 666 -C llwa-state || exit 1
ach -o 666 -C rlwa-state || exit 1
ach -o 666 -C amc-cmd || exit 1
ach -o 666 -C llwa-cmd || exit 1
ach -o 666 -C rlwa-cmd || exit 1
ach -o 666 -C waistd-cmd || exit 1
ach -o 666 -C waist-cmd || exit 1
ach -o 666 -C lgripper-state|| exit 1
ach -o 666 -C lgripper-cmd || exit 1
ach -o 666 -C rgripper-state|| exit 1
ach -o 666 -C rgripper-cmd || exit 1
ach -o 666 -C llwa_ft
ach -o 666 -C rlwa_ft
ach -o 666 -C torso-state
ach -o 666 -C torso-cmd
ach -o 666 -C event

achd -r pull 192.168.1.51 imu-data -d   
achd -u 10000 -r pull 192.168.1.51 waist-state -d
achd -u 10000 -r pull 192.168.1.51 amc-state -d 
achd -u 10000 -r pull 192.168.1.51 dynamixel-state -d
achd -r pull 192.168.1.51 cinder -d 
achd -u 10000 -r pull 192.168.1.51 llwa-state -d 
achd -u 10000 -r pull 192.168.1.51 rlwa-state -d 
achd -u 10000 -r pull 192.168.1.51 llwa_ft -d 
achd -u 10000 -r pull 192.168.1.51 rlwa_ft -d 
