# This message is for storing detection based measures such as score and the class id
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

float64 score # uncertainty of this detection
uint32 class_id # the unique class identifier
string class_name # the unique class name
string detection_id # the detection id (for tracking this will stay the same for the same object)