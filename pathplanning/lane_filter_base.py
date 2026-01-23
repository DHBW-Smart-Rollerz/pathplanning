import numpy as np
from numpy.polynomial import polynomial as pol


class LaneFilterBase:
    """Base for filtering lanes with useful functions to use."""

    min_x = 0.0
    max_x = 1.5

    def __init__(
        self, lane="unknown", buffer_size=10, diff_threshold=10.0, logger=None
    ):
        """
        Init Filter base class.

        Args:
            buffer_size (int, optional): Size of the buffer for previous results.
            diff_threshold (float, optional): Threshold for difference to consider a new fit significantly different.
        """
        self.lane = lane
        self.buffer_size = buffer_size
        self.diff_threshold = diff_threshold
        self.buffer = []
        self._logger = logger

    def fit(self, lane):
        """
        Fit lane data. Must be implemented by subclasses.

        Args:
            lane (dict): Detected lane information.

        Returns:
            list: List of points representing the fitted lane.
        """
        # Remove points from lane where x is within min_x and max_x
        if "points" in lane:
            filtered_points = [
                pt for pt in lane["points"] if (self.min_x <= pt[0] <= self.max_x)
            ]
            lane["points"] = filtered_points

    def update_buffer(self, result):
        """
        Update the buffer with new points.

        Args:
            result (dict): Newly fitted lane result. Result may be points or coeffs and intercept.
        """
        self.buffer.append(result)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

    def sample_points_from_poly(self, coeffs, num_samples=50):
        """
        Creates sample points along a polynomial curve with given coeffs.

        Args:
            coeffs (array-like): Coefficients of poly.

        Returns:
            list: List of points representing the fitted lane.
        """
        x_vals = np.linspace(self.min_x, self.max_x, num=num_samples)
        y_vals = np.polyval(
            coeffs[::-1], x_vals
        )  # change coeff order from low-high to high-low
        points = list(map(list, zip(x_vals, y_vals)))

        return points

    def get_weighted_buffer_average(self):
        """
        Calculate a weighted average on all items inside the buffer (newer frames have higher weight).

        Returns:
            array-like: Coefficients to use.
        """
        # Should never be accessed because buffer has to be filled before this function
        if not self.buffer:
            return None

        # Use weights so the path reacts quickly to turns
        weights = np.exp(np.linspace(0, 2, len(self.buffer)))
        weights /= weights.sum()

        # Calculate average based on weights
        stacked = np.vstack(self.buffer)
        avg_coeffs = np.average(stacked, axis=0, weights=weights)

        return avg_coeffs

    def compare_to_new_coeff(self, coeffs):
        """
        Compares newly fitted coeffs to the last result.

        Args:
            coeffs (array-like): Coefficients of newly fitted poly.

        Returns:
            bool: True if difference is smaller than threshold and by that valid.
        """
        if not self.buffer:
            return True

        diff = abs(coeffs[0] - self.buffer[-1][0])

        return diff < self.diff_threshold
