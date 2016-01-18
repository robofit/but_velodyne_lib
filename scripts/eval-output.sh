#! /bin/bash

function average {
	awk '{ sum += $1; n++ } END { print sum / n; }'
}

function min {
	sort -g | head -n1
}

function max {
	sort -g | tail -n1
}

if [ $# -ne 1 ]; then
	echo "Invalid arguments. Usage: ./eval-output.sh <output.txt>" >&2
	exit 1
fi

output=$1

tmp=$(mktemp)
grep "time: [0-9.]\+s" $output -o | grep "[0-9.]\+" -o > $tmp
min=$(min < $tmp)
max=$(max < $tmp)
avg=$(average < $tmp)
echo "Time: [$min; $max], average: $avg"

grep "iterations: [0-9]\+" $output -o | grep "[0-9]\+" -o > $tmp
min=$(min < $tmp)
max=$(max < $tmp)
avg=$(average < $tmp)
echo "Iterations: [$min; $max], average: $avg"
