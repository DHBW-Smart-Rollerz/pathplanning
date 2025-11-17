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
        poly = PolynomialFeatures(degree)

        x_poly = poly.fit_transform(x.reshape(-1, 1))

        base = RidgeCV()
        reg = RANSACRegressor(base, residual_threshold=0.05, min_samples=5).fit(
            x_poly, y
        )

        x_vis = np.linspace(x.min(), x.max(), 100)
        x_vis_poly = poly.transform(x_vis.reshape(-1, 1))
        y_vis = reg.predict(x_vis_poly)
        points = np.column_stack((x_vis, y_vis)).tolist()

        # use last result if new fit is significantly different
        if self.compare_results(points):
            points = self.last_results[-1]

        self.update_buffer(points)

        return points
