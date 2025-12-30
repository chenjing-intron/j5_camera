#./gpio.sh side1
i2cset -y -f 6 0x27 0x06 0xd600 w 
usleep 100000

./gpio.sh

usleep 100000

mount ./lib/ /system/lib/sensorlib/
#./udp-streamer-demo config_first_two/config_6pipe.json
#./udp-streamer-demo config_first_two/config_1pipe.json
./udp-streamer-demo config_first_two/config_4pipe.json

#/app/bin/camera_test/j5_camera_test.sh config_first_two/hb_j5dev.json 0 t1
#./vcs_test -m 4 -v ./config/vpm_config.json -p 10 -c ./config/hb_j5dev.json -i 0 -h /system/etc/vio_tool/dump_config.json -s 2147483647
