#! /bin/bash

if [ $# -ne 1 ]; then
	echo "Invalid arguments. Expected: $0 <results-dir>" >&2
	exit 1
fi

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
RES_DIR=$1

mkdir -p $RES_DIR/xz_errors
rm -rf $RES_DIR/xz_errors/speed_errors.txt

for f in $RES_DIR/../../data/odometry/poses/*.txt
do 
	seq=$(basename $f | cut -d"." -f1)
	echo $seq $($SCRIPT_DIR/evaluate.py $RES_DIR/data/$seq.txt $f $RES_DIR/xz_errors/$seq.err $RES_DIR/xz_errors/speed_errors.txt)
done | tee $RES_DIR/xz_errors/all.err |
	awk '{len+=$2;err+=$3*$2} END{print err/len}' | 
	tee $RES_DIR/xz_errors/avg.err
