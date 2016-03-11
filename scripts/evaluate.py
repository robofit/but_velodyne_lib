#! /usr/bin/env python

import sys
from math import sqrt

class XZ:
    def __init__(self, x, z):
        self.x = x
        self.z = z
    def __sub__(self, other):
        return XZ(other.x - self.x, other.z - self.z)
    def diff(self, other):
        return sqrt(((self.x - other.x) ** 2) + ((self.z - other.z) ** 2))
    def size(self):
        return sqrt(self.x ** 2 + self.z ** 2)
        
def get_xz_poses(filename):
    xz = []
    file = open(filename, "r")
    for line in file.readlines():
        pose = line.split()
        xz.append(XZ(float(pose[3]), float(pose[11])))
    return xz

def get_sq_error_fun(computed, gt):
    error_fun = []
    for i in range(1, len(gt)):
        gt_delta = gt[i] - gt[i-1]
        comp_delta = computed[i] - computed[i-1]
        sq_error = gt_delta.diff(comp_delta)
        error_fun.append(sq_error)
    return error_fun

def get_speed_fun(poses):
    speed_fun = []
    for i in range(1, len(gt)):
        pose_delta = poses[i] - poses[i-1]
        speed = pose_delta.size() / 0.1 * 3.6   # km/h
        speed_fun.append(speed)
    return speed_fun

def get_mean_fun(function, output_file):
    total_len = 0
    err_sum = 0
    a = 0
    for error in function:
        total_len += 1
        err_sum += error
        if output_file:
            output_file.write("%s\t%s\n" % (total_len, error))
    return err_sum / total_len
    
if len(sys.argv) != 5:
    sys.stderr.write("Invalid number of arguments, expected: <poses-computed> <poses-GT> <output-dst> <speeds-error-dst>\n")
    sys.exit(1)

computed = get_xz_poses(sys.argv[1])
gt = get_xz_poses(sys.argv[2])
output = open(sys.argv[3], "w")
speed_errors = open(sys.argv[4], "a")

if(len(computed) != len(gt)):
    sys.stderr.write("Number of computed poses (%s) != GT (%s)\n" % (len(computed), len(gt)))
    sys.exit(1)

errors = get_sq_error_fun(computed, gt)
speeds = get_speed_fun(gt)
sys.stdout.write("%s %s %s\n" % (len(gt), get_mean_fun(errors, output), get_mean_fun(speeds, None)))

summarize_per = 1;
sum_speed = 0.0;
sum_error = 0.0;
for i in range(len(errors)):
    if i % summarize_per == 0:
        speed_errors.write("%s\t%s\n" % (sum_speed/summarize_per, sum_error/summarize_per))
        sum_speed = 0.0;
        sum_error = 0.0;
    sum_error += errors[i]
    sum_speed += speeds[i]
