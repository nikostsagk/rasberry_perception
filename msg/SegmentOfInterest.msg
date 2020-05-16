# This message is for storing a segment of interest (i.e. segmentation results) in a generic format
#
# If the segment is only in 2D the z array should be left empty.
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

int32[] x  # Array of [x1 x2 ... xn] in image co-ordinates
int32[] y  # Array of [y1 y2 ... yn] in image co-ordinates
int32[] z  # Array of [z1 z2 ... zn] in image co-ordinates