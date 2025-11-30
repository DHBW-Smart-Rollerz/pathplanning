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

    def update_buffer(self, result):
        """
        Update the buffer with new points.

        Args:
            result (dict): Newly fitted lane result. Result may be points or coeffs and intercept.
        """
        self.buffer.append(result)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

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

    def compare_polys(self, coeffs1, coeffs2):
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

        if self._logger is not None:
            self._logger.debug(
                f"{self.lane} curvature diff: {diff:.6f}, threshold: {self.diff_threshold}"
            )
        print(diff)
        return diff > self.diff_threshold

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
