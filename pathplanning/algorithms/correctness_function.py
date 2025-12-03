import numpy as np
from sklearn.metrics import mean_squared_error


def lane_fit_quality_score(
    y,
    y_pred,
    outlier_k=3.0,
    rmse_tol=0.15,
    medae_tol=0.10,
    maxres_tol=0.50,
    w_rmse=0.30,
    w_medae=0.25,
    w_maxres=0.15,
    w_inlier=0.30,
    weighted_thr=None,
    w_weighted=0.20,
    w_ignored=0.10,
):
    """
    Berechnet einen einzigen Correctness-Faktor im Bereich [0,1].
    0 = perfekt, 1 = schlecht.

    Diese Funktion vergleicht eine Punktewolke (y) mit einer gefitteten Spur (y_pred):
      - y: Punktewolke, shape (N, 2) oder (N, D) mit [x, y, ...] Koordinaten
      - y_pred: gefittete Spur, shape (M, 2) oder (M, D)

    Für jeden Punkt in y wird die euklidische Distanz zum nächsten Punkt in y_pred
    berechnet (nearest neighbor distance). Ausreißer werden erkannt.

    Die Funktion erkennt und ignoriert Ausreißer mittels MAD
    (Median Absolute Deviation). Metriken (RMSE, MedAE, MaxResidual)
    werden auf den korrekten Werten prozentual berechnet; zusätzlich wird die Inlier-
    ratio als Robustheitsmaß verwendet.

    Args:
        y: Punktewolke (N x D array), z.B. (N, 2) mit [x, y] pro Punkt
        y_pred: gefittete Spur (M x D array), z.B. (M, 2) mit [x, y] pro Punkt
        outlier_k: Schwellenfaktor für MAD-basiertes Outlier-Detect (default 3.0)
        *_tol: Toleranzen zum Normieren der Metriken auf [0,1]
        w_*: Gewichte (müssen zusammen approx. 1 ergeben)
        weighted_thr: Schwelle für "große Residuen unter Inliers" (default: rmse_tol)
        w_weighted: Gewicht für Penalty bei großen Inlier-Residuen (default 0.20)
        w_ignored: Gewicht für Benefit bei ignorierten Ausreißern (default 0.10)
    """
    y = np.asarray(y, dtype=float)
    y_pred = np.asarray(y_pred, dtype=float)

    if y.size == 0 or y_pred.size == 0:
        # undefiniert / schlechter Fit
        return 1.0, {"error": "empty_inputs"}

    # ensure both are 2D
    if y.ndim == 1:
        y = y.reshape(-1, 1)
    if y_pred.ndim == 1:
        y_pred = y_pred.reshape(-1, 1)

    if y.shape[1] != y_pred.shape[1]:
        # mismatch in dimensionality
        return 1.0, {"error": "dimension_mismatch"}

    # Compute nearest neighbor distances: for each point in y,
    # find the closest point in y_pred and compute euclidean distance
    from scipy.spatial.distance import cdist

    distances = cdist(y, y_pred, metric="euclidean")  # shape (N, M)
    resid_vec = np.min(distances, axis=1)  # shape (N,) - min distance per point in y

    # --- robust outlier detection via MAD ------------------
    med = np.median(resid_vec)
    mad = np.median(np.abs(resid_vec - med))
    if mad < 1e-9:
        # fallback to std if no dispersion
        mad = np.std(resid_vec) + 1e-9

    inlier_mask = np.abs(resid_vec - med) <= (outlier_k * mad)
    inlier_ratio = float(np.mean(inlier_mask))
    ignored_ratio = float(1.0 - inlier_ratio)  # Anteil der ignorierten Ausreißer

    # restrict arrays to inliers for robust metric computation
    if np.any(inlier_mask):
        resid_in = resid_vec[inlier_mask]
    else:
        resid_in = resid_vec  # nothing to ignore

    # --- 1) Einzelmetriken (auf Inliers) -------------------------
    rmse = float(np.sqrt(np.mean(resid_in**2)))
    medae = float(np.median(np.abs(resid_in)))
    # use a robust max -> 95th percentile to reduce leftover single-point effect
    maxres = float(np.percentile(np.abs(resid_in), 95))

    # --- 2) Normalisierung auf [0,1] -------------------------
    s_rmse = min(1.0, rmse / rmse_tol)
    s_medae = min(1.0, medae / medae_tol)
    s_maxres = min(1.0, maxres / maxres_tol)
    s_inlier = 1.0 - inlier_ratio

    # --- 2b) weighted inlier penalty (große Residuen unter Inliers) ---
    if weighted_thr is None:
        weighted_thr = rmse_tol

    if np.any(inlier_mask):
        weighted_inlier_ratio = float(np.mean(resid_in > weighted_thr))
    else:
        # keine Inliers => starker Penalty
        weighted_inlier_ratio = 1.0

    # --- 3) Aggregation -------------------------
    # Klassische Fehlerkomponenten + Penalty für große Residuen unter Inliers
    # - Benefit für ignorierte Ausreißer (wird subtrahiert)
    CF = (
        w_rmse * s_rmse
        + w_medae * s_medae
        + w_maxres * s_maxres
        + w_inlier * s_inlier
        + w_weighted * weighted_inlier_ratio
        - w_ignored * ignored_ratio
    )

    # clamp to [0,1]
    CF = float(max(0.0, min(1.0, CF)))

    return CF, {  # Verschiedene Qualitätsmetriken
        "RMSE_inliers": rmse,  # Root Mean Square Error, robuste Schätzung (kleiner, desto besser)
        "MedAE_inliers": medae,  # Median Absolute Error, Maß für typische Abweichung, weniger Ausreißer-beeinflusst
        "MaxResidual95_inliers": maxres,  # maximum der unteren 95% der Residuen (robuster Maximalwert, gegen Ausreißer geschützt)
        "InlierRatio": inlier_ratio,  # + Anteil der nicht-Ausreißer (0.0: Alle Ausreißer, 1.0: Alle korrekt)
        "IgnoredRatio": ignored_ratio,  # + Ignorierte Ausreißer (0.0 (nichts) - 1.0: Alle ignoriert)
        "WeightedInlierRatio": weighted_inlier_ratio,  # Anteil der weit entfernten Punkte unter den nicht-Ausreißern
        "Score_RMSE": s_rmse,  # rmse normalisiert auf [0,1] - je kleiner, desto besser
        "Score_MedAE": s_medae,  # medae normalisiert auf [0,1] - je kleiner, desto besser
        "Score_MaxResidual": s_maxres,  # maxres normalisiert auf [0,1] - je kleiner, desto besser
        "Score_Inlier": s_inlier,  # Anteil der nicht-Ausreißer normalisiert auf [0,1] (0.0: Alle korrekt, 1.0: Alle Ausreißer)
        "outlier_k": outlier_k,  # Schwellenfaktor für Reproduzierbarkeit der Ausreißer/Ergebnisse
        "mad": float(mad),  # Median Absolute Deviation (Streuungsmaß der Punkte)
    }


"""
if __name__ == "__main__":
    # small usage example
    # Punktewolke (z.B. von der Lane Detection)
    y = np.array([[0.0, 1.0],
                  [0.1, 2.0],
                  [0.2, 1.5],
                  [0.3, 2.1],
                  [0.4, 2.0]])     # letzter Punkt = Ausreißer

    # gefittete Spur (z.B. von Ridge oder Kalman)
    y_pred = np.array([[0.0, 1.0],
                       [0.1, 2.0],
                       [0.2, 1.4],
                       [0.3, 2.0],
                       [0.4, 2.0],
                       [3.2, 1.0]])   # guter Fit bis auf den Ausreißer

    score, metrics = correctness_score(y, y_pred)
    print(f"Correctness Score: {score:.3f}")
    print(f"Metrics: {metrics}")"""
