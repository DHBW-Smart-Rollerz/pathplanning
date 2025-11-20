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
        self.last_results = []
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
        self.last_results.append(result)
        if len(self.last_results) > self.buffer_size:
            self.last_results.pop(0)

    # Comparison functions from here onwards for robustness over multiple frames

    def compare_points(self, new_points):
        """
        Compare new points to last result.

        Args:
            new_points (list): Newly fitted lane points.

        Returns:
            bool: True if difference exceeds threshold, False otherwise.
        """
        if not self.last_results:
            return False
        last = np.array(self.last_results[-1])
        curr = np.array(new_points)
        diff = np.mean(np.linalg.norm(last - curr, axis=1))
        return diff > self.diff_threshold

    def compare_polys(self, new_coeffs):
        """
        Compare new polynomial coefficients to last result.

        Args:
            new_coeffs (array-like): Coefficients of the newly fitted polynomial.
            new_intercept (float): Intercept of the newly fitted polynomial.

        Returns:
            bool: True if difference exceeds threshold, False otherwise.
        """
        if not self.last_results:
            return new_coeffs

        last_coeffs = self.last_results[-1]

        p_new = np.poly1d(np.asarray(new_coeffs))
        p_last = np.poly1d(last_coeffs)

        dp_new = np.polyder(p_new)
        dp_last = np.polyder(p_last)

        c_new = np.asarray(dp_new.c, dtype=float)
        c_last = np.asarray(dp_last.c, dtype=float)

        # pad shorter coeff array with zeros on the left (highest-order side)
        if c_new.size > c_last.size:
            c_last = np.pad(c_last, (c_new.size - c_last.size, 0), mode="constant")
        elif c_last.size > c_new.size:
            c_new = np.pad(c_new, (c_last.size - c_new.size, 0), mode="constant")

        diff = np.mean(np.abs(c_new - c_last))

        # prepare full polynomial coeff arrays (not derivatives) and pad to same length
        coeff_new = np.asarray(p_new.c, dtype=float)
        coeff_last = np.asarray(p_last.c, dtype=float)

        # decision based on diff
        if diff < self.diff_threshold:
            # accept new coefficients
            result_coeffs = coeff_new
            msg = "accept new coeffs"
        elif diff < self.diff_threshold * 1.5:
            # smoothly blend between new and old: weight goes from 1 at diff=0.2 to 0 at diff=0.5
            weight_new = (0.5 - diff) / (0.5 - 0.2)
            blended = weight_new * coeff_new + (1.0 - weight_new) * coeff_last

            # robustness: detect extreme per-coefficient outliers and fallback to median of the two
            stacked = np.vstack([coeff_new, coeff_last])
            med = np.median(stacked, axis=0)
            mad = np.median(np.abs(stacked - med), axis=0) + 1e-8
            diff_coeff = np.abs(coeff_new - coeff_last)
            outlier_mask = diff_coeff > (5.0 * mad)
            if np.any(outlier_mask):
                blended[outlier_mask] = med[outlier_mask]

            result_coeffs = blended
            msg = f"blended coeffs (weight_new={weight_new:.3f})"
        else:
            # keep old coefficients
            result_coeffs = coeff_last
            msg = "REALLY BAD, reuse old coeffs"

        if self._logger:
            self._logger.info(f" {self.lane}: {msg}, derivative diff={diff:.4f}")

        return result_coeffs
