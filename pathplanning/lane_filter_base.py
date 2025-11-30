import numpy as np


class LaneFilterBase:
    """Base for filtering lanes with useful functions to use."""

    def __init__(
        self, lane="unknown", buffer_size=10, diff_threshold=10.0, logger=None
    ):
        """
        Init Filter base class.

        Args:
            buffer_size (int, optional): Size of the buffer for previous results.
            diff_threshold (float, optional): Threshold for difference to consider a new fit significantly different.
        """
        self.lane = lane
        self.buffer_size = buffer_size
        self.diff_threshold = diff_threshold
        self.buffer = []
        self._logger = logger

    def fit(self, lane):
        """
        Fit lane data. Must be implemented by subclasses.

        Args:
            lane (dict): Detected lane information.

        Returns:
            list: List of points representing the fitted lane.
        """
        pass

    # Comparison functions from here onwards for robustness over multiple frames

    def compare_polys_with_full_buffer(self):
        """
        Compare all poly coefficients in the whole buffer.

        Returns:
            array-like: Coefficients to use.
        """
        if not self.buffer:
            return None

        buf = np.asarray(self.buffer)
        N, D = buf.shape

        inliers = np.ones(N, dtype=bool)

        # Process each coefficient independently
        for dim in range(D):
            coeffs = buf[:, dim]

            median = np.median(coeffs)
            mad = np.median(np.abs(coeffs - median))  # Median Absolute Deviation

            # Avoid divide-by-zero
            if mad < 1e-9:
                continue

            # Compute deviations (scaled)
            deviation = np.abs(coeffs - median) / mad

            # Mark outliers
            inliers &= deviation < self.diff_threshold

        # If all rejected, fall back to median
        if not np.any(inliers):
            return np.median(buf, axis=0)

        # Compute average using only inliers
        avg_coeffs = np.mean(buf[inliers], axis=0)

        self._logger.info(f"{self.lane} Lane Inliers Count: {np.sum(inliers)} / {N}")

        return avg_coeffs

    def compare_points(self, new_points):
        """
        Compare new points to last result.

        Args:
            new_points (list): Newly fitted lane points.

        Returns:
            bool: True if difference exceeds threshold, False otherwise.
        """
        if not self.buffer:
            return False
        last = np.array(self.buffer[-1])
        curr = np.array(new_points)
        diff = np.mean(np.linalg.norm(last - curr, axis=1))

        return diff > self.diff_threshold
