from abc import abstractmethod

from scipy.signal import butter, lfilter


class Filter:
    """
    Abstract class to ensure that every supported filter needs to implement the filter method.

    :method filter: Abstract method to filter the given data.
    """

    @abstractmethod
    def filter(self, data):
        pass


class Butterworth(Filter):
    """
    Implementation for a Butterworth filter.

    :param order: The order of the filter (default is 4).
    :param cutoff: The cutoff frequency of the filter (default is 10).
    :param fs: The sampling frequency of the data (default is 60).
    """

    def __init__(self, order=4, cutoff=10, fs=60):
        self.order = order
        self.cutoff = cutoff
        self.fs = fs

        self.b, self.a = butter(self.order, cutoff / (0.5 * fs), btype="low")

    def filter(self, data: list):
        """
        Filters the given data using a Butterworth filter.

        :param data: The data to be filtered.

        :return: The filtered data.
        """
        return lfilter(self.b, self.a, data)
