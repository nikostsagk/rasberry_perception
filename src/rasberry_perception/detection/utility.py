#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

from collections import deque
from timeit import default_timer as timer

from threading import Thread, Event
from rasberry_perception.detection.interfaces.compat import Queue
import rospy


class WorkerTaskQueue(Queue):
    """Class to allow offloading tasks such as publishing and visualisation"""
    def __init__(self, num_workers=1):
        Queue.__init__(self)
        self._stop = Event()
        self.num_workers = num_workers
        self._start_workers()

    def stop(self):
        """Signal the workers to stop. Please call WorkerTaskQueue.join() after this."""
        self._stop.set()

    def add_task(self, task, args, **kwargs):
        """Add a task to the worker task queue

        Args:
            task: Function/task to call with args/kwargs pair by one of the workers
            args: Tuple of arguments to unpack into task call (task(*args, **kwargs))
            **kwargs: Any other keyword args will be passed to the function (task(*args, **kwargs))

        """
        self.put((task, args, kwargs))

    def _start_workers(self):
        for _ in range(self.num_workers):
            thread = Thread(target=self._worker)
            thread.daemon = True
            thread.start()

    def _worker(self):
        while not rospy.is_shutdown() and not self._stop.is_set():
            task, args, kwargs = self.get()
            task(*args, **kwargs)
            self.task_done()


class FunctionTime:
    """Utility class containing different methods for benchmarking functions"""
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
