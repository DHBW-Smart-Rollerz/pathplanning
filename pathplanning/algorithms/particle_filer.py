import numpy as np

from pathplanning.lane_filter_base import LaneFilterBase


class ParticleFilter(LaneFilterBase):
    """
    Fit a lane curve using a simple particle filter over polynomial coefficients.

    Contract:
    - Input: lane (dict) with key "points" -> list of [x, y] in meters.
    - Output: numpy.ndarray shape (num_output_points, 2) with [[x, y], ...]

    Approach (summary):
    - Each particle represents a vector of polynomial coefficients (highest-first), degree `degree`.
    - Initialize particles by fitting polynomials to random subsets of points, add noise.
    - Iterate: compute particle weights from MSE against observed points, resample, jitter.
    - Return a dense set of points sampled from the averaged (weighted) polynomial.

    This implementation is lightweight and dependency-free (numpy only).
    """

    def fit(
        self,
        lane,
        degree=2,
        num_particles=500,
        num_iters=6,
        num_samples=200,
        noise_scale=0.1,
    ):
        """
        Fit.

        Args:
            lane (dict): Detected lane information.
            degree (int): Polynomial degree.
            num_particles (int): Number of particles.
            num_iters (int): Number of filter iterations.
            num_samples (int): Number of output samples along the x-range.
            noise_scale (float): Scale of noise for particle jittering.

        Returns:
            np.ndarray: Array of shape (M, 2) with smoothed lane points.
        """
        points = lane.get("points", [])
        if len(points) < degree + 1:
            return np.empty((0, 2))

        pts = np.array(points, dtype=float)
        x_obs = pts[:, 0]
        y_obs = pts[:, 1]

        # Normalize x to improve numerical stability (shift & scale)
        x_mean = x_obs.mean()
        x_std = x_obs.std() if x_obs.std() > 0 else 1.0
        x_norm = (x_obs - x_mean) / x_std

        # Helper: evaluate polynomial coeffs (highest-first) on x values
        def eval_polys(coefs, x_vals):
            # coefs: (P, deg+1)
            # returns (P, len(x_vals))
            return (
                np.polyval(coefs, x_vals[:, None]).T
                if False
                else np.asarray([np.polyval(c, x_vals) for c in coefs])
            )

        # Initialize particles by fitting random subsets
        rng = np.random.RandomState(0)
        particles = np.zeros((num_particles, degree + 1), dtype=float)
        for i in range(num_particles):
            # sample minimal number of points needed (degree+1)
            try:
                idx = rng.choice(len(x_norm), degree + 1, replace=False)
                coeffs = np.polyfit(x_norm[idx], y_obs[idx], deg=degree)
            except Exception:
                # fallback to global fit
                coeffs = np.polyfit(x_norm, y_obs, deg=degree)
            # add some noise
            particles[i] = coeffs + rng.normal(scale=noise_scale, size=coeffs.shape)

        # Particle filter iterations
        for it in range(num_iters):
            # compute predicted y for each particle on observed x
            preds = np.asarray([np.polyval(p, x_norm) for p in particles])  # (P, N)
            # compute MSE per particle
            mse = np.mean((preds - y_obs[None, :]) ** 2, axis=1)
            # convert to weights (avoid underflow)
            sigma = np.median(np.sqrt(mse)) + 1e-6
            weights = np.exp(-0.5 * (mse / (sigma**2 + 1e-9)))
            weights_sum = weights.sum()
            if weights_sum <= 0 or not np.isfinite(weights_sum):
                weights = np.ones_like(weights)
                weights_sum = weights.sum()
            weights /= weights_sum

            # resample according to weights
            indices = rng.choice(
                num_particles, size=num_particles, replace=True, p=weights
            )
            particles = particles[indices]

            # jitter particles (small gaussian noise)
            jitter_scale = noise_scale * (0.8**it)
            particles += rng.normal(scale=jitter_scale, size=particles.shape)

            # occasional random particle injection to avoid local minima
            if it % 3 == 2:
                n_rand = max(1, num_particles // 50)
                particles[:n_rand] = np.polyfit(x_norm, y_obs, deg=degree) + rng.normal(
                    scale=noise_scale, size=(n_rand, degree + 1)
                )

        # Final weighted average of particles (recompute weights one last time)
        preds = np.asarray([np.polyval(p, x_norm) for p in particles])
        mse = np.mean((preds - y_obs[None, :]) ** 2, axis=1)
        sigma = np.median(np.sqrt(mse)) + 1e-6
        weights = np.exp(-0.5 * (mse / (sigma**2 + 1e-9)))
        weights /= weights.sum()
        avg_coefs = np.average(particles, axis=0, weights=weights)

        # Produce output curve on a dense x range (in original units)
        x_out = np.linspace(x_obs.min(), x_obs.max(), num_samples)
        x_out_norm = (x_out - x_mean) / x_std
        y_out = np.polyval(avg_coefs, x_out_norm)

        return np.column_stack((x_out, y_out))
