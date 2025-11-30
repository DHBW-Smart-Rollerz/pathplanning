from lane_msgs.msg import Lane


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
            if (point.x / 1000) <= 1.0
        ],
        "detected": Lane.detected,
    }


class LaneFilterManager:
    """Management class for lane filtering algorithms."""

    def __init__(self, lane_filters):
        """
        Initialize LaneFilter with specified lane filtering algorithms.

        Args:
            lane_filters (dict): Dictionary mapping lane names to their respective filter instances.
        """
        self.lane_filters = lane_filters

    def fit(self, lane_detection_result):
        """
        Fit all lanes using their respective filters.

        Args:
            lane_detection_result (LaneDetectionResult): Detected lane information.

        Returns:
            dict: Fitted lane points for each lane.
        """
        coordinates = {"left": [], "center": [], "right": []}

        for lane_name in coordinates.keys():
            lane = getattr(lane_detection_result, lane_name)
            serialized_lane = serialize_lane(lane)

            serialized_lane["points"] = [
                p for p in serialized_lane["points"] if p[0] <= 3
            ]

            if len(serialized_lane["points"]) >= 20 and lane.detected:
                points = self.lane_filters[lane_name].fit(serialized_lane)
            else:
                points = []

            coordinates[lane_name] = points

        return coordinates
