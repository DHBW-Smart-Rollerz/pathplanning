import numpy as np
from sklearn.linear_model import Ridge
from sklearn.preprocessing import PolynomialFeatures


def ridge(lane):
    """
    Ridge Regression algorithm.

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

    reg = Ridge().fit(x_poly, y)

    # Sample points along the x-range for visualization
    x_vis = np.linspace(min(x.min(), x.max()), max(x.min(), x.max()), 500)
    x_vis_poly = poly.transform(x_vis.reshape(-1, 1))
    y_vis = reg.predict(x_vis_poly)
    points = np.column_stack((x_vis, y_vis)).tolist()

    return points
