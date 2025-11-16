import numpy as np
from scipy.interpolate import splev, splprep

from pathplanning.lane_filter_base import LaneFilterBase


class BSpline(LaneFilterBase):
    """Fit a cubic B-spline to lane points."""

    def fit(self, lane, num_samples=500, degree=3):
        """
        Fit.

        Args:
            lane (dict): Detected lane information.
            num_samples (int): Number of samples along the spline.
            degree (int): Spline degree (default 3).

        Returns:
            list: List of points representing the fitted lane.
        """
        points = lane.get("points", [])
        if len(points) < degree + 1:
            return []
        x = np.array([p[0] for p in points])
        y = np.array([p[1] for p in points])
        try:
            tck, u = splprep([x, y], s=0, k=degree)
            u_new = np.linspace(0, 1, num_samples)
            out = splev(u_new, tck)
            return list(map(list, zip(out[0], out[1])))
        except Exception:
            return []
