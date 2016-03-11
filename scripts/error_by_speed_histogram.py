#! /usr/bin/env python

import sys
import math
import new

class Bin:
    def __init__(self):
        self.count = 0
        self.value = 0.0
    
    def add(self, addition):
        self.count += 1
        self.value += addition
    
    def avg(self):
        if(self.count == 0):
            return 0
        else:
            return self.value / self.count


def process_line(line):
    tokens = line.split("\t")
    speed = float(tokens[0])
    error = float(tokens[1])
    return (speed, error)

MAX_SPEED = 400

if(len(sys.argv) > 1):
    bin_size = int(sys.argv[1])
else:
    bin_size = 10;

if(len(sys.argv) > 2):
    win_size = int(sys.argv[2])
else:
    win_size = 3;
    
histogram = [Bin() for i in range(MAX_SPEED/bin_size)]
speed_errors = map(process_line, sys.stdin.readlines())

for begin in range(len(speed_errors) - 3):
    window = speed_errors[begin:(begin+win_size)]
    sum_speeds = sum(map(lambda x: x[0], window))
    sum_errors = sum(map(lambda x: x[1], window))
    bin_index = int(math.floor(sum_speeds / win_size / bin_size))
    histogram[bin_index].add(sum_errors / win_size)

speed = 0
for bin in histogram:
    sys.stdout.write("%s\t%s\t%s\n" % (speed, bin.count, bin.avg()))
    speed += bin_size
