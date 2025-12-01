import numpy as np
from lane_msgs.msg import Lane

min_x = 0.0
max_x = 1.5
lane_distances = 0.6  # meters between lanes


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
            if (point.x / 1000) <= max_x and (point.x / 1000) >= min_x
        ],
        "detected": Lane.detected,
    }


class LaneFilterManager:
    """Management class for lane filtering algorithms."""

    def __init__(self, lane_filters, buffer_size=5, diff_threshold=1.0, logger=None):
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
        bad_fits = []

        # fitting lanes to get coeffs
        self._logger.info("Fitting lanes...")
        for lane_name in self.buffer.keys():
            lane = getattr(lane_detection_result, lane_name)
            serialized_lane = serialize_lane(lane)

            if len(serialized_lane["points"]) >= 20 and lane.detected:
                coeffs = self.lane_filters[lane_name].fit(serialized_lane)

                if self.buffer[lane_name]:
                    _, mean_deviation = (
                        self.compare_coeffs(coeffs, self.buffer[lane_name][-1])
                        if self.buffer[lane_name]
                        else (None, None)
                    )
                    if mean_deviation > self.diff_threshold:
                        bad_fits.append(lane_name)
                        # self._logger.info(f"{lane_name} lane fit rejected due to high deviation: {mean_deviation:.3f}. Comparing to other lanes.")

                self.update_buffer(lane_name, coeffs)

        if len(bad_fits) > 0:
            _, mean_deviation_12 = self.compare_coeffs(
                self.buffer["left"][-1],
                self.buffer["center"][-1],
                include_lowest_order=False,
            )
            _, mean_deviation_13 = self.compare_coeffs(
                self.buffer["left"][-1],
                self.buffer["right"][-1],
                include_lowest_order=False,
            )
            _, mean_deviation_23 = self.compare_coeffs(
                self.buffer["center"][-1],
                self.buffer["right"][-1],
                include_lowest_order=False,
            )

            self._logger.info(
                f"Inter-lane deviations: L1-L2: {mean_deviation_12:.3f}, L1-L3: {mean_deviation_13:.3f}, L2-L3: {mean_deviation_23:.3f}"
            )

            max_deviation = 0.25
            if "left" in bad_fits:
                if (
                    mean_deviation_12 < max_deviation
                    and mean_deviation_13 < max_deviation
                ):
                    self._logger.info(
                        "Keeping left lane because it is consistent with other lanes."
                    )
                    self.buffer["left"] = [self.buffer["left"][-1]]
                else:
                    self.buffer["left"].pop()
                    self._logger.info("Removing left lane fit from buffer.")

            if "center" in bad_fits:
                if (
                    mean_deviation_12 < max_deviation
                    and mean_deviation_23 < max_deviation
                ):
                    self._logger.info(
                        "Keeping center lane because it is consistent with other lanes."
                    )
                    self.buffer["center"] = [self.buffer["center"][-1]]
                else:
                    self.buffer["center"].pop()
                    self._logger.info("Removing center lane fit from buffer.")

            if "right" in bad_fits:
                if (
                    mean_deviation_13 < max_deviation
                    and mean_deviation_23 < max_deviation
                ):
                    self._logger.info(
                        "Keeping right lane because it is consistent with other lanes."
                    )
                    self.buffer["right"] = [self.buffer["right"][-1]]
                else:
                    self.buffer["right"].pop()
                    self._logger.info("Removing right lane fit from buffer.")

        coordinates = {"left": [], "center": [], "right": []}

        # generating points from coeffs
        for lane_name in self.buffer.keys():
            if not self.buffer[lane_name]:
                continue

            coeffs = np.mean(self.buffer[lane_name], axis=0)
            xs = np.linspace(0, max_x, 50)
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

    def compare_coeffs(self, coeffs1, coeffs2, include_lowest_order=True):
        """
        Compare two sets of polynomial coefficients.

        Args:
            coeffs1 (list): First set of polynomial coefficients.
            coeffs2 (list): Second set of polynomial coefficients.

        Returns:
            bool: True if the coefficients differ significantly, False otherwise.
        """
        coeffs1 = coeffs1.copy()
        coeffs2 = coeffs2.copy()
        if not include_lowest_order:
            coeffs1[-1] = 0
            coeffs2[-1] = 0
        x_eval = np.linspace(0, max_x, 50)
        y1 = np.polyval(coeffs1, x_eval)
        y2 = np.polyval(coeffs2, x_eval)

        diff = np.abs(y1 - y2)
        max_deviation = np.max(diff)
        mean_deviation = np.mean(diff)
        # self._logger.info(f"Max deviation: {max_deviation}, Mean deviation: {mean_deviation}")
        return max_deviation, mean_deviation
