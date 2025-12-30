i2cset -y -f 6 0x27 0x06 0xd600 w 
usleep 100000

./gpio.sh

