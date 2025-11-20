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

        coef = reg.estimator_.coef_
        intercept = reg.estimator_.intercept_
        poly_coeffs = np.concatenate((coef[::-1], [intercept]))

        # use last result if new fit is significantly different
        poly_coeffs = self.compare_polys(poly_coeffs)
        self.update_buffer(poly_coeffs)

        xs = np.linspace(x.min(), x.max(), 100)
        ys = np.polyval(poly_coeffs, xs)

        points = list(zip(xs, ys))

        return points
