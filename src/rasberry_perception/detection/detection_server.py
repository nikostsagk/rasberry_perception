#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Executable for running the detection service server backend from rosrun

import argparse

from rasberry_perception.detection import Server


def _default_arg_parser(args=None):
    parser = argparse.ArgumentParser(description='Run the detection server.')
    parser.add_argument('--backend', type=str, help="Which backend to use.", default=None)
    parsed_args, unknown = parser.parse_known_args() #this is an 'internal' method

    # Add unrecognised args as kwargs for passing the detection server
    unknown_parser = argparse.ArgumentParser()
    for arg in unknown:
        if arg.startswith(("-", "--")):
            unknown_parser.add_argument(arg, type=str)
    unknown_args = unknown_parser.parse_args(unknown)
    unknown_kwargs = {a: getattr(unknown_args, a) for a in vars(unknown_args)} if len(vars(unknown_args)) else None

    return parsed_args, unknown_kwargs


def __detection_server_runner():
    # Command line arguments should always over ride ros parameters
    args, args_kwargs = _default_arg_parser()

    server = Server(backend=args.backend, backend_kwargs=args_kwargs)
    server.run()


if __name__ == '__main__':
    __detection_server_runner()
