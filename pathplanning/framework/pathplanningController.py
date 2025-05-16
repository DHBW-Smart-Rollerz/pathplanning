import math
from collections import deque

import numpy as np
from scipy.optimize import minimize

LANE_DIST = 350


class PPController:
    """
    Enhanced Path Planning Controller with improved trajectory stability.

    Features:
    - Savitzky-Golay filtering for smooth trajectory generation
    - Higher-order polynomial fitting for better curve representation
    - Rate limiting to prevent sudden changes
    - Exponential moving average smoothing
    - Robust outlier detection and handling
    """

    def __init__(
        self, parameter_callback: callable, debug: bool = False, logger: callable = None
    ):
        """
        Initialize PPController with enhanced smoothing parameters.

        Args:
            parameter_callback (callable): Get the parameter to the corresponding key.
            debug (bool): Enable debug mode for detailed logging.
            logger (callable): Logger object for error tracking.
        """
        self.remote_state = 0
        self.parameter_callback = parameter_callback
        self._debug = debug
        self.logger = logger

        # Smoothing configuration parameters
        self.poly_degree = 2  # Higher degree for better curve fitting
        self.smoothing_alpha = 0.6  # EMA smoothing factor (lower = smoother)
        # self.rate_limit = 0.2  # Maximum allowed rate of change
        self.max_history = 50  # Number of previous coefficients to store
        self._coef_min = [-0.01, -10, -1e6]  # Minimum coefficient value
        self._coef_max = [0.01, 10, 1e6]  # Maximum coefficient value
        self._max_variance = 0.03  # Maximum allowed variance y coordinate

        self.reset()

    def reset(self):
        """Reset controller state and trajectory history."""
        self._curr_var = 0
        self.prev_lanes = deque(maxlen=self.max_history)
        self.prev_lanes.append([0] * (self.poly_degree + 1))

    def _filter_unique(self, points):
        """
        Filter unique x-coordinates and remove outliers.

        Args:
            points (list): List of (x, y) coordinate pairs.

        Returns:
            list: Filtered points without duplicates and outliers.
        """
        # Sort points by x-coordinate
        sorted_points = sorted(points, key=lambda p: p[0])

        # Remove duplicates while preserving order
        seen_x = set()
        filtered_points = []
        for point in sorted_points:
            if point[0] not in seen_x:
                seen_x.add(point[0])
                filtered_points.append(point)

        # Remove statistical outliers
        if len(filtered_points) > 3:
            y_values = np.array([p[1] for p in filtered_points])
            z_scores = np.abs((y_values - np.mean(y_values)) / np.std(y_values))
            return [p for p, z in zip(filtered_points, z_scores) if z < 3]

        return filtered_points

    def _polyfit_coefficients(self, coordinates):
        """
        Perform polynomial fitting with enhanced stability.

        Args:
            coordinates (list): List of (x, y) coordinate pairs.

        Returns:
            ndarray: Polynomial coefficients.
        """
        if not coordinates or len(coordinates) < 3:
            raise ValueError("Insufficient points for polynomial fitting")

        x = np.array([point[0] for point in coordinates])
        y = np.array([point[1] for point in coordinates])

        try:
            return np.polyfit(x, y, self.poly_degree)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Polynomial fitting failed: {e}")
            raise ValueError("Polynomial fitting failed")

    def _rate_polynom(self, coefficients, var: float):
        """
        Smooth coefficient changes with rate limiting and EMA.

        Args:
            coefficients (list): New polynomial coefficients.
            leftorright (bool): True for left lane, False for right lane.

        Returns:
            list: Smoothed coefficients.
        """
        prev_coeffs = np.array(list(self.prev_lanes))

        if self.remote_state == 1 or not prev_coeffs.any():
            self.prev_lanes.append(coefficients)
            return coefficients

        alpha = 1 - (100 / var) if var > 0 else 1
        # Apply rate limiting
        smoothed_coeffs = alpha * np.array(coefficients).flatten() + (
            1 - alpha
        ) * np.mean(prev_coeffs, axis=0)

        # Clip coefficients to prevent extreme values
        smoothed_coeffs = np.clip(
            smoothed_coeffs,
            self._coef_min,
            self._coef_max,
        )

        self.prev_lanes.append(smoothed_coeffs)
        return smoothed_coeffs

    def _calc_variance(self, points: np.ndarray):
        """
        Calculate variance of y-coordinates.

        Args:
            points (np.ndarray): List of (x, y) coordinate pairs.

        Returns:
            float: Variance of y-coordinates.
        """
        if len(points) < 2:
            return 0
        return np.var(points[:, 1])

    def start_main_process(
        self, left_lane_points: list, center_lane_points: list, right_lane_points: list
    ):
        """
        Process lane points and generate smooth trajectory coefficients.

        Args:
            left_lane_points (list): Points defining left lane.
            center_lane_points (list): Points defining center lane.
            right_lane_points (list): Points defining right lane.

        Returns:
            tuple: (left_coefficients, right_coefficients)
        """
        try:
            # Calculate lane coefficients
            left_coeffs, right_coeffs = self._get_lane_coefficients(
                left_lane_points, center_lane_points, right_lane_points
            )

            return left_coeffs, right_coeffs

        except Exception as e:
            if self.logger:
                self.logger.error(f"Error in main process: {e}")
            return self._shift_left(self.prev_lanes[0]), self._shift_right(
                self.prev_lanes[0]
            )

    def _get_lane_coefficients(self, left_points, center_points, right_points):
        """
        Get lane coefficients.

        Args:
            points1: Points for the first set of lane points.
            points2: Points for the second set of lane points.

        Returns:
            list: List of lane coefficients.
        """
        # unique_points1 = self._filter_unique(points1)
        # unique_points2 = self._filter_unique(points2)
        left_side = self._polyfit_coefficients(left_points)
        right_side = self._polyfit_coefficients(right_points)
        center = self._polyfit_coefficients(center_points)
        var = np.mean(
            [
                self._calc_variance(np.array(left_points)),
                self._calc_variance(np.array(right_points)),
                self._calc_variance(np.array(center_points)),
            ]
        )
        middle_coefficients = np.mean([left_side, center, right_side], axis=0)
        rated = self._rate_polynom(middle_coefficients, var)
        return self._shift_left(rated), self._shift_right(rated)

    def _shift_left(self, coefficients):
        """
        Shift polynomial coefficients to the left (y-axis).

        Args:
            coefficients (list): Polynomial coefficients.

        Returns:
            list: Shifted polynomial coefficients.
        """
        shifted = coefficients.copy()
        shifted[-1] -= LANE_DIST
        return shifted

    def _shift_right(self, coefficients):
        """
        Shift polynomial coefficients to the right (y-axis).

        Args:
            coefficients (list): Polynomial coefficients.

        Returns:
            list: Shifted polynomial coefficients.
        """
        shifted = coefficients.copy()
        shifted[-1] += LANE_DIST
        return shifted

    def ref_point_controller(self, coefficients, est_vec):
        """
        Determines reference points for the controller based on the provided polynomial coefficients.

        Args:
            coefficients (list): List of coefficients representing the polynomial.
            est_vec (list): List of estimated vectors.

        Returns:
            tuple: Tuple containing (x, y, theta) representing the reference point coordinates and angle.
        """
        # pose = np.array([est_vec[0], est_vec[1]])
        # heading = est_vec[2]
        result = minimize(lambda x: np.polyval(coefficients, x), x0=est_vec[0])
        x = result.x[0]
        y = np.polyval(coefficients, x)

        theta = np.arctan(np.polyval(np.polyder(coefficients), x))
        return x, y, theta
