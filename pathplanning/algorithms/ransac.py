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

        x = np.array([point[0] for point in lane["points"]]).reshape(-1, 1)
        y = np.array([point[1] for point in lane["points"]])

        base = PolyfitBase()
        reg = RANSACRegressor(base, residual_threshold=0.25, min_samples=5).fit(x, y)

        coeffs = reg.estimator_.coeffs  # order is highest to lowest

        if self.compare_new_coeff_derivative(coeffs):
            self.update_buffer(coeffs)
        else:
            coeffs = self.buffer[-1]
            self._logger.debug(f"{self.lane} lane has big deviation, using last result")

            self.buffer.pop(0)  # "reset" when last 5 frames where denied

        # increase smoothness by calculating average
        coeffs = self.get_weighted_buffer_average()

        # generate sample points
        x_vals = np.linspace(self.min_x, self.max_x, 50)
        y_vals = reg.predict(x_vals.reshape(-1, 1))

        points = list(map(list, zip(x_vals, y_vals)))

        return points


from sklearn.metrics import mean_squared_error  # temp


class PolyfitBase:
    """Base for RANSAC using polynomial fitting."""

    def __init__(self, degree=3):
        """
        Initialize PolyfitBase.

        Args:
            degree (int): Degree of polynomial.
        """
        self.degree = degree

    def fit(self, X, y):
        """
        Fit function.

        Args:
            X (array-like): X-values.
            y (array-like): Y-values.
        """
        self.coeffs = np.polyfit(X.ravel(), y, self.degree)

    def get_params(self, deep=False):
        """
        Get parameters.

        Args:
            deep (bool): Whether to return deep parameters.
        """
        return {"degree": self.degree}

    def set_params(self, **parameters):
        """
        Set parameters.

        Args:
            **parameters: Parameters to set.
        """
        for parameter, value in parameters.items():
            setattr(self, parameter, value)
        return self

    def predict(self, X):
        """
        Predict function.

        Args:
            X (array-like): X-values.

        Returns:
            array-like: Predicted Y-values.
        """
        poly_eqn = np.poly1d(self.coeffs)
        y_hat = poly_eqn(X.ravel())
        return y_hat

    def score(self, X, y):
        """
        Score function.

        Args:
            X (array-like): X-values.
            y (array-like): Y-values.
        """
        return mean_squared_error(y, self.predict(X))
