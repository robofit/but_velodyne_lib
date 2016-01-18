#! /bin/bash

if [ $# -ne 2 ]
then
	echo "Wrong number of arguments. Usage: $0 <kitti-velodyne-seq-dir> <output-dir>" >&2
	exit 1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
extract_vfh=$DIR/../bin/extract-vfh

kitti_seq=$1
output_dir=$2

for seq in $kitti_seq/*
do
	seq_no=$(echo $seq | rev | cut -d"/" -f1 | rev)
	if [ $(nproc) -ge 8 ]; then
		$extract_vfh $output_dir/$seq_no-vfh.pcd $(ls $seq/velodyne/*.bin | sort | xargs) &
	else
		$extract_vfh $output_dir/$seq_no-vfh.pcd $(ls $seq/velodyne/*.bin | sort | xargs)
	fi
done 2>&1 | tee $output_dir/log.txt
