#! /bin/bash

if [ $# -ne 2 ]
then
	echo "Wrong number of arguments. Usage: $0 <binary> <kitti-velodyne-seq-dir>" >&2
	exit 1
fi

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

binary=$1
kitti_seqences=$2

output=$(mktemp --suffix -output.txt)
logs=$(mktemp --suffix -stderr.txt)
echo "Output will be written to $output, stderr to $logs"

export PATH=$PWD:$PATH
seq_no=04
seq=$kitti_seqences/04
$binary $(ls $seq/velodyne/*.bin | sort | head -n100 | xargs) 2> $logs | tee $output

rm -f $seq/velodyne/covariances.yaml $seq/velodyne/poses.graph

find $kitti_seqences -name '*.transform' | while read t
do
	rm $t
done

expectation=0.0388
echo Expected error: $expectation
echo Computed error: $($SCRIPTS_DIR/evaluate.py $output $SCRIPTS_DIR/data/04-first-100-gt.txt out speeds | cut -f2 -d" ")
rm -f out speeds
