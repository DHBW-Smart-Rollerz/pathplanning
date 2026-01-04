import numpy as np
from sklearn.linear_model import RANSACRegressor, RidgeCV
from sklearn.preprocessing import PolynomialFeatures

from pathplanning.lane_filter_base import LaneFilterBase


class RidgeRansac(LaneFilterBase):
    """Ransac filter using RidgeCV as base."""

    def fit(self, lane):
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

        if self.compare_new_coeff_derivative(coeffs):
            self.update_buffer(coeffs)
        else:
            coeffs = self.buffer[-1]
            self._logger.debug(f"{self.lane} lane has big deviation, using last result")

            self.buffer.pop(0)  # "reset" when last 5 frames where denied

        # increase smoothness by calculating average
        coeffs = self.get_weighted_buffer_average()

        points = self.sample_points_from_poly(coeffs=coeffs)

        return points
