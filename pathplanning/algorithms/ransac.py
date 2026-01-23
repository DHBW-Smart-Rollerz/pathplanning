import numpy as np
from numpy.polynomial import polynomial as pol
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
            return [], []

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

        if self.compare_to_new_coeff(coeffs):
            self.update_buffer(coeffs)
        else:
            coeffs = self.buffer[-1]
            self._logger.debug(f"{self.lane}: using last result")

            self.buffer.pop(0)  # "reset" when last 5 frames where denied
            if len(self.buffer) == 0:
                self._logger.debug(f"{self.lane}: buffer empty, reset")
                self.update_buffer(coeffs)
        # increase smoothness by calculating average
        coeffs = self.get_weighted_buffer_average()

        points = self.sample_points_from_poly(coeffs=coeffs)

        return coeffs, points


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
        self.coeffs = pol.polyfit(X.ravel(), y, self.degree)

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
        return pol.polyval(X.ravel(), self.coeffs)

    def score(self, X, y):
        """
        Score function.

        Args:
            X (array-like): X-values.
            y (array-like): Y-values.
        """
        return mean_squared_error(y, self.predict(X))
