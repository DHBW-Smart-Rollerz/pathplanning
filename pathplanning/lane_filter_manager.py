import numpy as np
from lane_msgs.msg import Lane

max_x = 1.5


def serialize_lane(Lane: Lane):
    """
    Serialize lane information.
    TODO: Should be moved to utils.

    Arguments:
        Lane -- Detected lane information.

    Returns:
        dict -- Serialized lane information.
    """
    return {
        "points": [
            [point.x / 1000, point.y / 1000]
            for point in Lane.points
            if (point.x / 1000) <= max_x
        ],
        "detected": Lane.detected,
    }


class LaneFilterManager:
    """Management class for lane filtering algorithms."""

    def __init__(self, lane_filters, buffer_size=5, diff_threshold=10.0, logger=None):
        """
        Initialize LaneFilter with specified lane filtering algorithms.

        Args:
            lane_filters (dict): Dictionary mapping lane names to their respective filter instances.
        """
        self.lane_filters = lane_filters
        self.buffer = {lane: [] for lane in lane_filters.keys()}
        self.buffer_size = buffer_size
        self.diff_threshold = diff_threshold
        self._logger = logger

    def fit(self, lane_detection_result):
        """
        Fit all lanes using their respective filters.

        Args:
            lane_detection_result (LaneDetectionResult): Detected lane information.

        Returns:
            dict: Fitted lane points for each lane.
        """
        coeff_list = {"left": [], "center": [], "right": []}

        for lane_name in coeff_list.keys():
            lane = getattr(lane_detection_result, lane_name)
            serialized_lane = serialize_lane(lane)

            serialized_lane["points"] = [
                p for p in serialized_lane["points"] if p[0] <= 3
            ]

            if len(serialized_lane["points"]) >= 20 and lane.detected:
                coeffs = self.lane_filters[lane_name].fit(serialized_lane)
                coeff_list[lane_name] = coeffs

        coordinates = {"left": [], "center": [], "right": []}

        for lane_name in coeff_list.keys():
            coeffs = coeff_list[lane_name]
            if len(coeffs) > 0:
                xs = np.linspace(-0.5, max_x, 50)
                ys = np.polyval(coeffs, xs)
                points = list(zip(xs, ys))
                coordinates[lane_name] = points

        return coordinates

    def update_buffer(self, lane_name, result):
        """
        Update the buffer with new points.

        Args:
            result (dict): Newly fitted lane result. Result may be points or coeffs and intercept.
        """
        self.buffer[lane_name].append(result)
        if len(self.buffer[lane_name]) > self.buffer_size:
            self.buffer[lane_name].pop(0)

    def compare_polys(self, coeffs1, coeffs2, diff_threshold=10.0):
        """
        Compare new polynomial coefficients to last result.

        Args:
            new_coeffs (array-like): Coefficients of the newly fitted polynomial.
            new_intercept (float): Intercept of the newly fitted polynomial.

        Returns:
            bool: True if difference is inside threshold, False otherwise.
        """
        c1 = np.asarray(coeffs1, dtype=float)
        c2 = np.asarray(coeffs2, dtype=float)

        # Align lengths by padding the shorter (assumes coeffs are highest-degree-first as from np.polyfit)
        if c1.size != c2.size:
            if c1.size > c2.size:
                c2 = np.pad(c2, (c1.size - c2.size, 0), mode="constant")
            else:
                c1 = np.pad(c1, (c2.size - c1.size, 0), mode="constant")

        try:
            p1 = np.poly1d(c1)
            p2 = np.poly1d(c2)

            dp1 = p1.deriv(1)
            ddp1 = p1.deriv(2)
            dp2 = p2.deriv(1)
            ddp2 = p2.deriv(2)

            # sample x over a reasonable range in front of the vehicle
            x = np.linspace(0.0, 30.0, 61)

            # curvature kappa = |y''| / (1 + y'^2)^(3/2)
            denom1 = (1.0 + dp1(x) ** 2) ** 1.5
            denom2 = (1.0 + dp2(x) ** 2) ** 1.5

            # avoid divide-by-zero
            denom1 = np.maximum(denom1, 1e-12)
            denom2 = np.maximum(denom2, 1e-12)

            kappa1 = np.abs(ddp1(x)) / denom1
            kappa2 = np.abs(ddp2(x)) / denom2

            diff = float(np.mean(np.abs(kappa1 - kappa2)))
        except Exception:
            # Fallback: use coefficient Euclidean distance if curvature computation fails
            diff = float(np.linalg.norm(c1 - c2))

        self._logger.debug(f"curvature diff: {diff:.6f}")

        return diff > diff_threshold
