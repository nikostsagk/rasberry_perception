# This message is for storing segmentation masks in a generic format
int16[] x  # Array of [x1 x2 ... xn] in image co-ordinates
int16[] y  # Array of [y1 y2 ... yn] in image co-ordinates
float64 score
int16 class_id