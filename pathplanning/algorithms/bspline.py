import numpy as np
import patsy
import statsmodels.api as sm
from pyclothoids import Clothoid
from scipy.interpolate import splev, splprep


def bspline(lane, num_samples=500, degree=3):
    """
    Fit a cubic B-spline to lane points.

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


def bibspline(lane, knots=None, degree=3, num_samples=100):
    """
    Fit a cubic B-spline regression to lane points using patsy and statsmodels.

    Args:
        lane (dict): Detected lane information.
        knots (list): List of knot positions for the spline.
        degree (int): Spline degree (default 3).
        num_samples (int): Number of samples for output curve.

    Returns:
        list: List of points representing the fitted lane.
    """
    points = lane.get("points", [])
    if len(points) < degree + 1:
        return []
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    if knots is None:
        # Use quantiles for knots if not provided
        knots = np.quantile(x, [0.25, 0.5, 0.75]).tolist()
    # Transform x using B-spline basis
    transformed_x = patsy.dmatrix(
        f"bs(x, knots={knots}, degree={degree}, include_intercept=False)",
        {"x": x},
        return_type="dataframe",
    )
    # Fit GLM model
    cs = sm.RLM(y, transformed_x, M=sm.robust.norms.HuberT()).fit()
    # Predict for dense x range
    x_pred = np.linspace(x.min(), x.max(), num_samples)
    transformed_x_pred = patsy.dmatrix(
        f"bs(x_pred, knots={knots}, degree={degree}, include_intercept=False)",
        {"x_pred": x_pred},
        return_type="dataframe",
    )
    y_pred = cs.predict(transformed_x_pred)
    return np.column_stack((x_pred, y_pred)).tolist()
