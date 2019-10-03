#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

from collections import deque
from timeit import default_timer as timer


class FunctionTime:
    def __init__(self, smooth_window=30, log_function=None):
        self.counter = 0
        if log_function is None:
            log_function = print
        self.log_function = log_function
        self.func_times = deque(maxlen=smooth_window)

    def interval_logger(self, interval):
        interval = interval * (0 < interval)
        self.counter = self.counter * (interval > self.counter)

        def function_time_decorator(method):
            def timed(*args, **kwargs):
                ts = timer()
                result = method(*args, **kwargs)
                self.func_times.append(timer() - ts)
                self.counter += 1
                if interval == self.counter:
                    self.log_function("{} {:.2f}ms ({:.2f} fps)".format(method.__name__, *self.__get_times()))
                    self.counter = 0
                return result
            return timed
        return function_time_decorator

    def logger(self, method):
        def timed(*args, **kwargs):
            ts = timer()
            result = method(*args, **kwargs)
            self.func_times.append(timer() - ts)
            self.log_function("{} {:.2f}ms ({:.2f} fps)".format(method.__name__, *self.__get_times()))
            return result
        return timed

    def __get_times(self):
        time_avg = sum(self.func_times) / len(self.func_times)
        ms, fps = time_avg * 1000, 1. / time_avg
        return ms, fps


function_timer = FunctionTime()
