#! /bin/bash

SDIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

for i in $(seq 0 4); do
	[ -f 00000$i.bin ] || wget --no-check-certificate https://www.fit.vutbr.cz/~ivelas/files/kitti-sample/00000$i.bin
done

odometry=$SDIR/../bin/collar-lines-odom
preview=$SDIR/../bin/show-kitti-poses

$odometry 000000.bin 000001.bin 000002.bin 000003.bin 000004.bin > estimated.poses
$preview estimated.poses 000000.bin 000001.bin 000002.bin 000003.bin 000004.bin
