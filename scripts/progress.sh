#! /bin/bash

if [ $# -ne 1 ]; then
	echo "Invalid arguments. Usage: ./progress.sh <out-dir-of-progress>" >&2
fi

total=43552

pushd $1
	processed=$(wc $(ls 0*.txt 1*.txt 2*.txt) | tail -n1 | grep -o '[0-9]\+' | head -n1)
	echo "Done: $(echo $processed.0/$total.0*100 | bc -l)%"
popd
