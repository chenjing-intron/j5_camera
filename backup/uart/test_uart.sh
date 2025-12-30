#!/bin/bash

counter=0

while [ $counter -lt 1000000000 ]
do
	echo "11111" > /dev/ttyS0
	echo "11111" > /dev/ttyS3
	sleep 0.001
	((counter++))
done


