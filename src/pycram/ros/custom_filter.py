from abc import abstractmethod

from scipy.signal import butter, lfilter


class CustomFilter:
    """
    Abstract class to ensure that every supported filter needs to implement the filter method
    """

    @abstractmethod
    def filter(self, data):
        pass


class Butterworth(CustomFilter):
    """
    Implementation for a Butterworth filter.
    """

    def __init__(self, order=4, cutoff=10, fs=60):
        self.order = order
        self.cutoff = cutoff
        self.fs = fs

        self.b, self.a = butter(self.order, cutoff / (0.5 * fs), btype='low')

    def filter(self, data: list):
        return lfilter(self.b, self.a, data)
