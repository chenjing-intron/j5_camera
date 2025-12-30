./0-power_off.sh
usleep 500000
 ./0-power_on.sh
 usleep 500000
 ./set-10.sh

echo 0x1e > /sys/class/vps/mipi_host2/param/ipi1_dt
echo 0x2c > /sys/class/vps/mipi_host2/param/ipi2_dt


./j5_camera_test.sh ./hb_j5dev.json t1 d3