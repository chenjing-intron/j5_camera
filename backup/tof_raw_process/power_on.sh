#!/bin/sh

#CN4:


#POC0_EN
echo 353 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio353/direction
echo 1 > /sys/class/gpio/gpio353/value


#CN2 96712 PWD_NB
echo 488 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio488/direction
echo 1 > /sys/class/gpio/gpio488/value
		
		
#CN2 1.0 EN
echo 505 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio505/direction
echo 1 > /sys/class/gpio/gpio505/value

#CN2 1.2 EN
echo 409 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio409/direction
echo 1 > /sys/class/gpio/gpio409/value

#CN2 1.8 EN
echo 503 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio503/direction
echo 1 > /sys/class/gpio/gpio503/value


#CN2 12v EN
echo 413 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio413/direction
echo 1 > /sys/class/gpio/gpio413/value
