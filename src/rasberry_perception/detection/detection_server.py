#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Executable for running the detection service server backend from rosrun


def __detection_server_runner():
    from rasberry_perception.detection import Server
    server = Server()
    server.run()


if __name__ == '__main__':
    __detection_server_runner()
