import math
from collections import deque

import numpy as np


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
        self._coef_min = [-0.006, -6, -1e6]  # Minimum coefficient value
        self._coef_max = [0.006, 6, 1e6]  # Maximum coefficient value
        self._max_variance = 0.1  # Maximum allowed variance y coordinate

        self.reset()

    def reset(self):
        """Reset controller state and trajectory history."""
        self._curr_var = 0
        self.prev_lanes = {
            "left": deque(maxlen=self.max_history),
            "right": deque(maxlen=self.max_history),
        }
        self.prev_lanes["left"].append([0] * (self.poly_degree + 1))
        self.prev_lanes["right"].append([0] * (self.poly_degree + 1))

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

    def _rate_polynom(self, coefficients, leftorright: bool, var: float):
        """
        Smooth coefficient changes with rate limiting and EMA.

        Args:
            coefficients (list): New polynomial coefficients.
            leftorright (bool): True for left lane, False for right lane.

        Returns:
            list: Smoothed coefficients.
        """
        side = "left" if leftorright else "right"
        prev_coeffs = np.array(list(self.prev_lanes[side]))

        if self.remote_state == 1 or not prev_coeffs.any():
            self.prev_lanes[side].append(coefficients)
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

        self.prev_lanes[side].append(smoothed_coeffs)
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
            left_coeffs = self._get_lane_coefficients(
                left_lane_points, center_lane_points, True
            )
            right_coeffs = self._get_lane_coefficients(
                center_lane_points, right_lane_points, False
            )

            return left_coeffs, right_coeffs

        except Exception as e:
            if self.logger:
                self.logger.error(f"Error in main process: {e}")
            return self.prev_lanes["left"][-1], self.prev_lanes["right"][-1]

    def _get_lane_coefficients(self, points1, points2, left_or_right: bool):
        """
        Get lane coefficients.

        Args:
            points1: Points for the first set of lane points.
            points2: Points for the second set of lane points.
            left_or_right (bool): True for left lane, False for right lane.

        Returns:
            list: List of lane coefficients.
        """
        unique_points1 = self._filter_unique(points1)
        unique_points2 = self._filter_unique(points2)
        left_side = self._polyfit_coefficients(unique_points1)
        right_side = self._polyfit_coefficients(unique_points2)
        middle_coefficients = self._middle_coefficients(left_side, right_side)
        var = np.mean(
            [
                self._calc_variance(np.array(points1)),
                self._calc_variance(np.array(points2)),
            ]
        )
        return self._rate_polynom(middle_coefficients, left_or_right, var)

    def _middle_coefficients(self, left_coeffs, right_coeffs):
        """
        Calculate middle coefficients from left and right lane coefficients.

        Args:
            left_coeffs: Coefficients for the left lane.
            right_coeffs: Coefficients for the right lane.

        Returns:
            list: List of middle coefficients.
        """
        if len(left_coeffs) != len(right_coeffs):
            raise ValueError("Coefficient lengths do not match")

        return [(l + r) / 2 for l, r in zip(left_coeffs, right_coeffs)]

    def ref_point_controller(self, coefficients):
        """
        Determines reference points for the controller based on the provided polynomial coefficients.

        Args:
            coefficients (list): List of coefficients representing the polynomial.

        Returns:
            tuple: Tuple containing (x, y, theta) representing the reference point coordinates and angle.
        """
        p = np.poly1d(coefficients)
        x = self.parameter_callback("trj_look_forward").value
        y = p(x)
        theta = -1 * math.atan(
            -2 * coefficients[0] * (y / 1000) - coefficients[1]
        )  # Tom fragen
        return x, y, theta
