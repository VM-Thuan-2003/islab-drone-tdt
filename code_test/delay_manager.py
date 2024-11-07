import time


class DelayManager:
    """Class to manage multiple delays

    Attributes:
        _delays (dict): A dictionary of delay names to their respective start times
    """

    __slots__ = ('_delays',)

    def __init__(self):
        """Initialize the DelayManager class"""
        self._delays = {}

    def start(self, name: str) -> None:
        """Start a delay

        Args:
            name (str): The name of the delay
        """
        self._delays[name] = time.time()

    def delay(self, name: str, delay_time: float = 0.1) -> bool:
        """Delay for the specified amount of time

        Args:
            name (str): The name of the delay
            delay_time (float): The amount of time to delay in seconds.

        Returns:
            bool: True if the delay is finished, False otherwise
        """
        current_time = time.time()
        if current_time - self._delays[name] >= delay_time:
            self._delays[name] = current_time
            return True
        return False

