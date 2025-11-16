import numpy as np
from sklearn.linear_model import HuberRegressor
from sklearn.preprocessing import PolynomialFeatures

from pathplanning.lane_filter_base import LaneFilterBase


class HuberRegression(LaneFilterBase):
    """Huber Regression algorithm."""

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

        try:
            reg = HuberRegressor(epsilon=2).fit(x_poly, y)
        except Exception:
            return []

        # Sample points along the x-range for visualization
        x_vis = np.linspace(x.min(), x.max(), 100)
        x_vis_poly = poly.transform(x_vis.reshape(-1, 1))
        y_vis = reg.predict(x_vis_poly)
        points = np.column_stack((x_vis, y_vis)).tolist()

        return points
