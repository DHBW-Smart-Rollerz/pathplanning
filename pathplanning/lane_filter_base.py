import numpy as np


class LaneFilterBase:
    """Base for filtering lanes with useful functions to use."""

    def __init__(self, buffer_size=10, diff_threshold=10.0, logger=None):
        """
        Init Filter base class.

        Args:
            buffer_size (int, optional): Size of the buffer for previous results.
            diff_threshold (float, optional): Threshold for difference to consider a new fit significantly different.
        """
        self.buffer_size = buffer_size
        self.diff_threshold = diff_threshold
        self.last_results = []
        self._logger = logger

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

    def compare_polys(self, new_coeffs, new_intercept):
        """
        Compare new polynomial coefficients to last result.

        Args:
            new_coeffs (array-like): Coefficients of the newly fitted polynomial.
            new_intercept (float): Intercept of the newly fitted polynomial.

        Returns:
            bool: True if difference exceeds threshold, False otherwise.
        """
        if not self.last_results:
            return False
        last_coeffs = self.last_results[-1]["coeffs"]
        last_intercept = self.last_results[-1]["intercept"]
        coeffs_diff = np.linalg.norm(np.array(last_coeffs) - np.array(new_coeffs))
        intercept_diff = abs(last_intercept - new_intercept)

        self._logger.info(int(coeffs_diff + intercept_diff))

        return (coeffs_diff + intercept_diff) > self.diff_threshold

    def update_buffer(self, result):
        """
        Update the buffer with new points.

        Args:
            result (dict): Newly fitted lane result. Result may be points or coeffs and intercept.
        """
        self.last_results.append(result)
        if len(self.last_results) > self.buffer_size:
            self.last_results.pop(0)
