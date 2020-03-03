#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com


def __detection_server_runner():
    """Executable for running the detection service server backend from rosrun"""
    from rasberry_perception.detection.service import Server
    server = Server()
    server.run()


if __name__ == '__main__':
    __detection_server_runner()
