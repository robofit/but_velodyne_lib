#! /bin/bash

#! /bin/bash

if [ $# -ne 3 ]
then
	echo "Wrong number of arguments. Usage: $0 <dataset-velodyne-dir> <transformations> <output-dir>" >&2
	exit 1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
find_loops=$DIR/../bin/find-loops-vfh
register_loops=$DIR/../bin/register-loops

velodyne_dataset=$1
transformations_dir=$2
output_dir=$3

set -x

for seq in $velodyne_dataset/sequences/*
do
	seq_no=$(echo $seq | rev | cut -d"/" -f1 | rev)
	
	features=$velodyne_dataset/vhf/$seq_no-vfh.pcd
	poses=$output_dir/$seq_no.txt
	matches=$output_dir/matches-$seq_no.txt
	covariances=$output_dir/covariances-$seq_no.yaml
	graph_file=$output_dir/graph-$seq_no.txt
	transformations=$transformations_dir/$seq_no/velodyne
	
	$find_loops $features $poses GT > $matches
	$register_loops $matches $poses $covariances $seq $transformations $graph_file 
done 2>&1 | tee $output_dir/loops-log.txt
