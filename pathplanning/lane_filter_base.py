import numpy as np
from numpy.polynomial import polynomial as pol


class LaneFilterBase:
    """Base for filtering lanes with useful functions to use."""

    def __init__(
        self,
        min_x,
        max_x,
        lane="unknown",
        buffer_size=10,
        diff_threshold=10.0,
        logger=None,
    ):
        """
        Init Filter base class.

        Args:
            min_x (float): Minimum x value for lane points to consider.
            max_x (float): Maximum x value for lane points to consider.
            lane (str, optional): Name of the lane being filtered. Defaults to "unknown".
            buffer_size (int, optional): Size of the buffer for previous results.
            diff_threshold (float, optional): Threshold for difference to consider a new fit significantly different.
            logger (logging.Logger, optional): Logger for debug messages.
        """
        self.lane = lane
        self.buffer_size = buffer_size
        self.diff_threshold = diff_threshold
        self.buffer = []
        self._logger = logger
        self.min_x = min_x
        self.max_x = max_x
        self.trials = []

    def fit(self, lane):
        """
        Filter lane data on x and y axis to ignore noise on image border. Can be used by subclasses.

        Args:
            lane (list): Detected lane points.

        Returns:
            list: List of points representing the fitted lane.
        """
        # Remove points from lane where x is within min_x and max_x
        outer_limit = 0.85
        filtered_points = [
            pt
            for pt in lane
            if (self.min_x <= pt[0] <= self.max_x)
            and (-outer_limit < pt[1] < outer_limit)
        ]
        return filtered_points

    def check_crossing_direction(self, coeffs, crossing_state):
        """
        Check if the fitted lane matches the desired crossing direction.

        Args:
            coeffs (array-like): Coefficients of the fitted polynomial.
            crossing_state (int): Desired crossing state (-1: left, 0: straight, 1: right).

        Returns:
            bool: True if the lane matches the desired direction, False otherwise.
        """
        p = np.polynomial.Polynomial(coeffs)
        y_start = p(self.min_x)
        y_end = p(self.max_x)

        slope = y_end - y_start  # positive is left, negative is right

        threshold = 0.3  # threshold for slope to consider it a turn

        # left turn
        if crossing_state == -1 and slope > threshold:
            return True

        # right turn
        elif crossing_state == 1 and slope < -threshold:
            return True

        return False

    def update_buffer(self, result):
        """
        Update the buffer with new coefficients.

        Args:
            result (dict): Newly fitted lane result.
        """
        self.buffer.append(result)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

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

        diff = abs(coeffs[-1] - self.buffer[-1][-1])

        return diff < self.diff_threshold
