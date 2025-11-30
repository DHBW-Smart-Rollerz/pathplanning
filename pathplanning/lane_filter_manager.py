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

                if self.buffer[lane_name] and not self.compare_polys(
                    coeffs, self.buffer[lane_name][-1]
                ):
                    self._logger.info("big change")
                    continue

                self.update_buffer(lane_name, coeffs)

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
