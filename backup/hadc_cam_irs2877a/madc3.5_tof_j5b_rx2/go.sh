usleep 500000
./reset_max96712_and_poc_in_j5b.sh
usleep 500000
#./j5_camera_test.sh ./hb_j5dev.json t1 d10000000000000

./j5_camera_test -c  ./hb_j5dev.json -t1 -d1 -o1 -p /map



