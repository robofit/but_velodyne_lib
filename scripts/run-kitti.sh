#! /bin/bash

if [ $# -ne 4 ]
then
	echo "Wrong number of arguments. Usage: $0 <binary> <kitti-velodyne-seq-dir> <output-dir> <git-ref>" >&2
	exit 1
fi

binary=$1
kitti_seq=$2
output_dir=$3
git_ref=$4

if [ $(nproc) -ge 8 ]; then
	parallel=true
else
	parallel=false
fi

mkdir -p $output_dir
echo "git ref: $git_ref" | tee $output_dir/notes.txt

export PATH=$PWD:$PATH
for seq in $kitti_seq/*
do
	seq_no=$(echo $seq | rev | cut -d"/" -f1 | rev)
	if [ $parallel == true ]; then
		$binary $(ls $seq/velodyne/*.bin | sort | xargs) > $output_dir/$seq_no.txt &
	else
		$binary $(ls $seq/velodyne/*.bin | sort | xargs) > $output_dir/$seq_no.txt
	fi
done 2>&1 | tee $output_dir/output.txt

wait

for seq in $kitti_seq/*
do
	seq_no=$(echo $seq | rev | cut -d"/" -f1 | rev)
	mv $seq/velodyne/covariances.yaml $output_dir/covariances-$seq_no.yaml
	mv $seq/velodyne/poses.graph $output_dir/poses-$seq_no.graph
done

find $kitti_seq -name '*.transform' | zip -@ $output_dir/transformations.zip
find $kitti_seq -name '*.transform' | while read t
do
	rm $t
done
