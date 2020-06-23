#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

from collections import deque
from timeit import default_timer as timer

from threading import Thread, Event
from rasberry_perception.compat import Queue
import rospy


class WorkerTaskQueue(Queue):
    """Class to allow offloading tasks such as publishing and visualisation"""
    def __init__(self, num_workers=1, max_size=30, discard=True):
        Queue.__init__(self, maxsize=max_size)
        self._stop = Event()
        self._discard = discard
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
        # If discard is true ignore jobs that can't be run right now
        if self._discard and self.qsize() == self.maxsize:
            return
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


class _FunctionTime:
    """ Decorator class for creating function timer objects

    Available functions are interval_logger and logger:

    .. code-block:: python

        import time
        from . import function_timer

        # Print the time every nth (5) run
        @function_timer.interval_logger(5)
        def time_me():
            time.sleep(1)

        # Print the time every run
        @function_timer.logger()
        def time_me_always():
            time.sleep(2)

        while True:
            time_me()
            time_me_always()

    Attributes:
        smooth_window: Number of past values to average the time over
        log_function: The function used to output the time (default is print)
    """
    def __init__(self, smooth_window=30, log_function=None):
        self.counter = 0
        if log_function is None:
            log_function = print
        self.log_function = log_function
        self.func_times = deque(maxlen=smooth_window)

    def copy(self):
        """Returns new FunctionTime object, allows objects to easily be duplicated in one line"""
        return _FunctionTime(self.func_times.maxlen, log_function=self.log_function)

    def interval_logger(self, interval):
        """ Decorator to log the function time every interval runs

        Args:
            interval: When to log the time (every interval runs)
        """
        # Always return new instance of interval logger
        return self.copy()._interval_logger(interval)

    def _interval_logger(self, interval):
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
        """ Decorator to log the function time every run"""
        return self.copy()._logger(method)

    def _logger(self, method):
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


function_timer = _FunctionTime()
