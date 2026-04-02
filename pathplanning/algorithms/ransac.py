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
        lane = super().fit(lane)

        if len(lane) < 10:
            return []

        x = np.array([point[0] for point in lane])
        y = np.array([point[1] for point in lane])

        degree = 3

        poly = PolynomialFeatures(degree, include_bias=False)

        x_poly = poly.fit_transform(x.reshape(-1, 1))

        base = RidgeCV()

        # residual_threshold = 0.15 # for "all in one" fitting
        residual_threshold = 0.25  # for seperated fitting
        reg = RANSACRegressor(
            base, residual_threshold=residual_threshold, min_samples=5
        ).fit(x_poly, y)

        self.trials.append(reg.n_trials_)

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
                    f"{self.lane}: not pointing to correct crossing direction. clearing..."
                )
                return []

        return coeffs
