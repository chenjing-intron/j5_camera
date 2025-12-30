#./gpio.sh side1
i2cset -y -f 6 0x27 0x06 0xd600 w 
usleep 100000

./gpio.sh

usleep 100000

mount ./lib/ /system/lib/sensorlib/
mount ./cam/ /system/etc/cam/

#./udp-streamer-demo config/config_6pipe.json
#./udp-streamer-demo config1/config_1pipe.json
#./udp-streamer-demo config_1300/config_4pipe.json

/app/bin/camera_test/j5_camera_test.sh config_1300/hb_j5dev.json 0 t1 d2
#./vcs_test -m 4 -v ./config/vpm_config.json -p 10 -c ./config/hb_j5dev.json -i 0 -h /system/etc/vio_tool/dump_config.json -s 2147483647
