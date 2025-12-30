i2cset -y -f 6 0x27 0x06 0xd600 w 
usleep 100000
./gpio.sh
usleep 100000
./set-60-80.sh


echo 0x1e > /sys/class/vps/mipi_host2/param/ipi1_dt
echo 0x2c > /sys/class/vps/mipi_host2/param/ipi2_dt

#./udp-streamer-demo  ./config_2pipe.json

./output ./hb_j5dev.json


