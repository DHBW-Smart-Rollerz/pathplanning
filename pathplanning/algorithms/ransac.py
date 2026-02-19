import numpy as np
from numpy.polynomial import polynomial as pol
from sklearn.linear_model import RANSACRegressor, RidgeCV
from sklearn.preprocessing import PolynomialFeatures

from pathplanning.lane_filter_base import LaneFilterBase


class Ransac(LaneFilterBase):
    """Ransac filter using RidgeCV as base."""

    def fit(self, lane, crossing_state):
        """
        Fit.

        Args:
            lane (dict): Detected lane information.

        Returns:
            list: List of points representing the fitted lane.
        """
        super().fit(lane)

        if len(lane["points"]) < 10:
            return []

        x = np.array([point[0] for point in lane["points"]])
        y = np.array([point[1] for point in lane["points"]])

        degree = 3

        poly = PolynomialFeatures(degree, include_bias=False)

        x_poly = poly.fit_transform(x.reshape(-1, 1))

        base = RidgeCV()
        reg = RANSACRegressor(base, residual_threshold=0.25, min_samples=5).fit(
            x_poly, y
        )

        coef_ = reg.estimator_.coef_
        intercept_ = reg.estimator_.intercept_
        coeffs = np.concatenate(([intercept_], coef_))  # lowest to highest order

        if self.compare_to_new_coeff(coeffs):
            self.update_buffer(coeffs)
        else:
            coeffs = self.buffer[-1]
            self._logger.debug(f"{self.lane}: using last result")

            self.buffer.pop(0)  # "reset" when last 5 frames where denied
            if len(self.buffer) == 0:
                self._logger.debug(f"{self.lane}: buffer empty, reset")
                # self.update_buffer(coeffs)
        # increase smoothness by calculating average
        if len(self.buffer) != 0:
            coeffs = self.get_weighted_buffer_average()

        if crossing_state != 0:
            if not self.check_crossing_direction(coeffs, crossing_state):
                self._logger.debug(
                    f"{self.lane}: direction in crossing is wrong way. Using predefined coeffs"
                )

                lowest_order = 0
                if self.lane == "left":
                    lowest_order = 0.7
                elif self.lane == "right":
                    lowest_order = -0.7
                coeffs = np.array([lowest_order, 0.30747138, 0.70643706, 0.74734169])
                self.buffer = (
                    []
                )  # reset buffer to prevent smoothing with wrong coeffs in next frames

        return coeffs
