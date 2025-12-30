#!/bin/sh

#CN4:


#POC0_EN
echo 353 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio353/direction
echo 0 > /sys/class/gpio/gpio353/value


#CN2 96712 PWD_NB
echo 488 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio488/direction
echo 0 > /sys/class/gpio/gpio488/value
		