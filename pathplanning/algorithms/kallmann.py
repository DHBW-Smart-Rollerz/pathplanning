import numpy as np


def kalman_filter(lane, num_samples=200, min_points=5):
    """
    Simple 2D Kalman filter for lane fitting.

    The function expects `lane` to be a dict with a "points" key containing
    an iterable of [x, y] pairs in meters. It returns a numpy array of shape
    (M, 2) with M == `num_samples` containing (x, y) points of the smoothed
    lane sampled uniformly across the x-range.

    Args:
        lane (dict): Detected lane information, must contain "points".
        num_samples (int): Number of output samples along the x-range.
        min_points (int): Minimum number of input points required.

    Returns:
        np.ndarray: Array of shape (M, 2) with smoothed lane points or
                    np.empty((0, 2)) if not enough input points.
    """
    points = lane.get("points", [])
    if len(points) < min_points:
        return np.empty((0, 2))

    pts = np.array(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 2:
        return np.empty((0, 2))

    # Sort by x to have a monotonic independent variable
    order = np.argsort(pts[:, 0])
    pts = pts[order]
    xs = pts[:, 0]
    zs = pts[:, 1]

    # State: [x, y, vx, vy]
    dt = 1.0
    F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    # Process and measurement noise (tunable)
    q_pos = 1e-4
    q_vel = 1e-3
    Q = np.diag([q_pos, q_pos, q_vel, q_vel])
    r_pos = max(1e-4, np.var(zs) * 0.5)
    R = np.diag([r_pos, r_pos])

    # Initial state: use first two points to estimate velocity
    x0 = xs[0]
    y0 = zs[0]
    if len(xs) >= 2 and xs[1] != xs[0]:
        vx0 = (xs[1] - xs[0]) / dt
        vy0 = (zs[1] - zs[0]) / dt
    else:
        vx0 = 0.0
        vy0 = 0.0

    state = np.array([x0, y0, vx0, vy0], dtype=float)
    P = np.diag([1.0, 1.0, 1.0, 1.0])

    filtered_positions = []

    I = np.eye(4)
    for xi, yi in zip(xs, zs):
        # Predict
        state = F @ state
        P = F @ P @ F.T + Q

        # Measurement update using current measurement
        z = np.array([xi, yi])
        y_tilde = z - (H @ state)
        S = H @ P @ H.T + R
        try:
            K = P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = P @ H.T @ np.linalg.pinv(S)

        state = state + K @ y_tilde
        P = (I - K @ H) @ P

        filtered_positions.append(state[:2].copy())

    filtered_positions = np.array(filtered_positions)

    # Ensure monotonic x for interpolation: if duplicates exist, average them
    fx = filtered_positions[:, 0]
    fy = filtered_positions[:, 1]
    # If not strictly increasing, coalesce duplicates by averaging
    unique_x, indices = np.unique(fx, return_index=False, return_inverse=True)
    if unique_x.size != fx.size:
        # average y for duplicate x
        ux = []
        uy = []
        for k in range(unique_x.size):
            mask = indices == k
            ux.append(np.mean(fx[mask]))
            uy.append(np.mean(fy[mask]))
        fx = np.array(ux)
        fy = np.array(uy)

    # If after processing we don't have enough unique points, return empty
    if fx.size < 2:
        return np.empty((0, 2))

    x_vis = np.linspace(float(np.min(xs)), float(np.max(xs)), num_samples)
    # Interpolate y values for dense x positions
    y_vis = np.interp(x_vis, fx, fy)

    return np.column_stack((x_vis, y_vis))
