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
        
def get_xz_poses(filename):
    xz = []
    file = open(filename, "r")
    for line in file.readlines():
        pose = line.split()
        xz.append(XZ(float(pose[3]), float(pose[11])))
    return xz

def get_sq_error_fun(computed, gt):
    error_fun = [(0.0, 0.0)]
    for i in range(1, len(gt)):
        #length = gt[i].diff(gt[i-1])
        length = 1
        gt_delta = gt[i] - gt[i-1]
        comp_delta = computed[i] - computed[i-1]
        sq_error = gt_delta.diff(comp_delta)
        error_fun.append((length, sq_error))
    return error_fun

def get_mean_error(err_function, output_file):
    total_len = 0
    err_integral = 0
    a = 0
    for error in err_function:
        total_len += error[0]
        err_integral += (a + error[1]) * error[0] / 2
        a = error[1]
        output_file.write("%s\t%s\n" % (total_len, a))
    return (total_len, err_integral / total_len)
    
if len(sys.argv) != 4:
    sys.stderr.write("Invalid number of arguments, expected: <poses-computed> <poses-GT>\n")
    sys.exit(1)

computed = get_xz_poses(sys.argv[1])
gt = get_xz_poses(sys.argv[2])
output = open(sys.argv[3], "w")

if(len(computed) != len(gt)):
    sys.stdout.write("Number of computed poses (%s) != GT (%s)\n" % (len(computed), len(gt)))
    sys.exit(1)
    
sys.stdout.write("%s %s\n" % (get_mean_error(get_sq_error_fun(computed, gt), output)))
