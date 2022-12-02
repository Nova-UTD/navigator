import time
from threading import Event, Thread

class BackgroundScheduler:
    """
    Repeat given <function> every <interval> seconds
    """

    def __init__(self, interval: float, function: callable, *args, **kwargs):
        """
        Initialization Function
        @param interval
        """
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.start = time.time()
        self.event = Event()
        self.thread = Thread(target=self._target)
        self.thread.start()

    def _target(self):
        while not self.event.wait(self._time):
            self.function(*self.args, **self.kwargs)

    @property
    def _time(self):
        return self.interval - ((time.time() - self.start) % self.interval)

    def stop(self):
        """
        Use to stop scheduler
        """
        self.event.set()
        self.thread.join()