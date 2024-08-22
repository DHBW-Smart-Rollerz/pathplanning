# Copyright (c) 2024 Smart Rollerz e.V.
# All rights reserved.

import math

import numpy as np


class PPController:
    """Path Planning Controller class."""

    def __init__(self, parameter_callback: callable, debug: bool = False):
        """
        Initialize PPController with parameter object and trajectory look forward.

        Args:
            parameter_callback (callable): Get the parameter to the corresponding key.
            debug (bool): Enable debug mode.
        """
        self.prev_lane_coefficients = {"left": [], "right": []}
        self.prev_left_lane_coeff = []
        self.prev_right_lane_coeff = []
        self.remote_state = 1
        self.parameter_callback = parameter_callback
        self._debug = debug

    def start_main_process(
        self, left_lane_points: list, center_lane_points: list, right_lane_points: list
    ):
        """
        Start the main processing.

        Args:
            left_lane_points (list): List of points for the left lane.
            center_lane_points (list): List of points for the center lane.
            right_lane_points (list): List of points for the right lane.

        Returns:
            tuple: Tuple containing left lane coefficients and right lane coefficients.
        """
        left_lane_coefficients = self._get_lane_coefficients(
            left_lane_points, center_lane_points, True
        )
        right_lane_coefficients = self._get_lane_coefficients(
            center_lane_points, right_lane_points, False
        )
        return left_lane_coefficients, right_lane_coefficients

    def reset_RC_MODE(self, state):
        """
        Reset the remote control mode.

        Arguments:
            state -- State of the remote control mode.
        """
        if state == 0:
            self.remote_state = 0
        else:
            self.prev_lane_coefficients = {"left": [], "right": []}
            self.prev_left_lane_coeff = []
            self.prev_right_lane_coeff = []
            self.remote_state = 1

    # Tools

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
        left_side = self._polyfit_coefficients(self._filter_unique(points1))
        right_side = self._polyfit_coefficients(self._filter_unique(points2))
        middle_coefficients = self._middle_coefficients(left_side, right_side)
        return self._rate_polynom(middle_coefficients, left_or_right)

    def _polyfit_coefficients(self, coordinates):
        """
        Polyfit coefficients.

        Args:
            coordinates: Coordinates to perform polyfit.

        Returns:
            array: Array of polyfit coefficients.
        """
        x = [point[0] for point in coordinates]
        y = [point[1] for point in coordinates]

        if not x or not y:
            raise ValueError("Both x and y vectors must be non-empty for polyfit.")

        return np.polyfit(y, x, 2)

    def _filter_unique(self, arr):
        """
        Filter unique values.

        Args:
            arr: Array to filter unique values.

        Returns:
            list: List of filtered unique values.
        """
        arr = sorted(arr, key=lambda item: item[0])

        unique_x_values = set()
        result_array = []
        for pair in arr:
            x_value = pair[0]

            # Überprüfe, ob der x-Wert bereits im Set ist
            if x_value not in unique_x_values:
                # Füge den x-Wert zum Set hinzu und füge das Paar zum Ergebnis-Array hinzu
                unique_x_values.add(x_value)
                result_array.append(pair)

        return result_array

    def _middle_coefficients(self, coefficients1, coefficients2):
        """
        Defines the middle coefficients between two sets of coefficients.

        Args:
            coefficients1: Coefficients from first set.
            coefficients2: Coefficients from second set.

        Returns:
            list: List of middled coefficients.
        """
        return [(c1 + c2) / 2 for c1, c2 in zip(coefficients1, coefficients2)]

    def _rate_polynom(self, coefficients, leftorright: bool):
        """
        Rates the provided polynomial coefficients based on their deviation from the previous lane coefficients.

        Args:
            coefficients (list): List of coefficients representing the polynomial.
            leftorright (bool): Boolean indicating whether the coefficients are for the left or right lane.

        Returns:
            list: Adjusted coefficients based on the deviation from the previous lane coefficients.
        """
        prev_lane_coeff = (
            self.prev_lane_coefficients["left"]
            if leftorright
            else self.prev_lane_coefficients["right"]
        )

        if self.remote_state == 1:
            return coefficients

        if not any(prev_lane_coeff):
            if leftorright:
                self.prev_lane_coefficients["left"] = coefficients
            else:
                self.prev_lane_coefficients["right"] = coefficients
            return coefficients

        deviation = np.linalg.norm(
            np.array(coefficients) - np.array(prev_lane_coeff)
        ) / np.linalg.norm(np.array(prev_lane_coeff))

        deviation = max(min(deviation, 0.90), 0)

        if self._debug:
            print(f"deviation: {deviation}")
            print(f"prev_lane_coeff: {prev_lane_coeff} - coefficients: {coefficients}")

        if -0.006 < coefficients[0] < 0.006 and -6 < coefficients[1] < 6:
            if 0.1 < deviation:
                coefficients = [
                    c * (1 - deviation) + p * deviation
                    for c, p in zip(coefficients, prev_lane_coeff)
                ]
            else:
                coefficients = [
                    c * 0.9 + p * 0.1 for c, p in zip(coefficients, prev_lane_coeff)
                ]
        else:
            coefficients = prev_lane_coeff

        if leftorright:
            self.prev_lane_coefficients["left"] = coefficients
        else:
            self.prev_lane_coefficients["right"] = coefficients

        return coefficients

    def draw_trajectory(self, coefficients):
        """
        Draws a trajectory based on the provided polynomial coefficients.

        Args:
            coefficients (list): List of coefficients representing the polynomial.

        Returns:
            list: List of tuples representing (x, y) coordinates of the trajectory.
        """
        p = np.poly1d(coefficients)  # Polynom erstellen

        y_fit = np.linspace(200, 630, 100)  # Werte für das Polynom
        x_fit = p(y_fit)

        return list(zip(x_fit, y_fit))

    def ref_point_controller(self, coefficients):
        """
        Determines reference points for the controller based on the provided polynomial coefficients.

        Args:
            coefficients (list): List of coefficients representing the polynomial.

        Returns:
            tuple: Tuple containing (x, y, theta) representing the reference point coordinates and angle.
        """
        p = np.poly1d(coefficients)
        y = self.parameter_callback("trj_look_forward").value
        x = p(y)
        theta = -1 * math.atan(
            -2 * coefficients[0] * (y / 1000) - coefficients[1]
        )  # Tom fragen
        return x, y, theta
