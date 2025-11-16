import numpy as np


class LaneFilterBase:
    """Base for filtering lanes with useful functions to use."""

    def __init__(self, buffer_size=5, diff_threshold=10.0):
        """
        Init Filter base class.

        Args:
            buffer_size (int, optional): Size of the buffer for previous results. Defaults to 5.
            diff_threshold (float, optional): Threshold for difference to consider a new fit significantly different. Defaults to 10.0.
        """
        self.buffer_size = buffer_size
        self.diff_threshold = diff_threshold
        self.last_results = []

    def fit(self, lane):
        """
        Fit lane data. Must be implemented by subclasses.

        Args:
            lane (dict): Detected lane information.

        Returns:
            list: List of points representing the fitted lane.
        """
        pass

    def compare_results(self, new_points):
        """
        Compare new fit to last result.

        Args:
            new_points (list): Newly fitted lane points.

        Returns:
            bool: True if difference exceeds threshold, False otherwise.
        """
        if not self.last_results:
            return False
        last = np.array(self.last_results[-1])
        curr = np.array(new_points)
        diff = np.mean(np.linalg.norm(last - curr, axis=1))
        return diff > self.diff_threshold

    def update_buffer(self, points):
        """
        Update the buffer with new points.

        Args:
            points (list): Newly fitted lane points.
        """
        self.last_results.append(points)
        if len(self.last_results) > self.buffer_size:
            self.last_results.pop(0)
