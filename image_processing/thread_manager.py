"""
ThreadManager Class

This class is used to manage a pool of worker threads. It provides methods
to start the threads, add tasks to the queue, wait for all tasks to be
completed, and stop the threads.

Example Usage:
    tm = ThreadManager(4)
    tm.start_threads()
    tm.add_task(my_func, arg1, arg2, kwarg1=1, kwarg2=2)
    tm.add_task(my_func, arg1, arg2, kwarg1=1, kwarg2=2)
    tm.wait_for_completion()
    tm.stop_threads()
"""

import threading
from queue import Queue
import time

class ThreadManager:
    def __init__(self, num_threads):
        """
        Initialize the thread manager

        :param num_threads: The number of worker threads to create
        """
        self.num_threads = num_threads  # Number of worker threads
        self.queue = Queue(maxsize=num_threads)  # Queue to hold tasks
        self.threads = []  # List to keep track of threads

    def worker(self):
        """
        The worker function that each thread will run. It will
        continuously pull tasks from the queue and execute them.
        """
        while True:
            task = self.queue.get()
            if task is None:
                break
            func, args, kwargs = task
            try:
                func(*args, **kwargs)
            except Exception as e:
                print(f"Error occurred: {e}")
            self.queue.task_done()

    def start_threads(self):
        """
        Initialize and start the worker threads
        """
        for _ in range(self.num_threads):
            thread = threading.Thread(target=self.worker)
            thread.daemon = True  # Daemon threads will exit when the program exits
            thread.start()
            self.threads.append(thread)

    def add_task(self, func, *args, **kwargs):
        """
        Add a task to the queue

        :param func: The function to be executed
        :param args: The arguments to pass to the function
        :param kwargs: The keyword arguments to pass to the function
        """
        self.queue.put((func, args, kwargs), block=False)

    def wait_for_completion(self):
        """
        Block until all tasks are done
        """
        self.queue.join()

    def stop_threads(self):
        """
        Stop all threads by sending None to each thread
        """
        for _ in range(self.num_threads):
            self.queue.put(None, block=False)
        for thread in self.threads:
            thread.join()

