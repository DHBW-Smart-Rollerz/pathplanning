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
        x = np.array([point[0] for point in lane["points"]])
        y = np.array([point[1] for point in lane["points"]])

        degree = 3

        poly = PolynomialFeatures(degree, include_bias=False)

        x_poly = poly.fit_transform(x.reshape(-1, 1))

        base = RidgeCV()
        reg = RANSACRegressor(base, residual_threshold=0.25, min_samples=5).fit(
            x_poly, y
        )

        # flipping array and adding intercept so coeffs are highest to lowest order
        coef_ = reg.estimator_.coef_
        intercept_ = reg.estimator_.intercept_
        coeffs = np.concatenate((coef_[::-1], [intercept_]))
        self.update_buffer(coeffs)

        x_vals = np.linspace(self.min_x, self.max_x, num=50)
        y_vals = np.polyval(coeffs, x_vals)
        points = zip(x_vals, y_vals)

        # use average of last results
        # coeffs = self.compare_polys_with_full_buffer()

        return points
