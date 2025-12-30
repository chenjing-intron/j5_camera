
i2cset -y -f 6 0x27 0x06 0xd600 w
killall uart_client
./uart_client /dev/ttyS3 9600 &
