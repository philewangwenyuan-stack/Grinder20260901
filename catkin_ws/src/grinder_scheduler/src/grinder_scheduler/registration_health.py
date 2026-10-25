"""Explain existing readiness gates without changing their thresholds."""
import math


def explain(quality, covariance, data_valid, xy_max, yaw_max, fitness_max,
            inlier_min, correspondences_min):
    reasons = []
    if not data_valid:
        reasons.append('data_invalid_or_stale')
    if len(covariance) < 36 or any(
            not math.isfinite(float(covariance[i])) or not 0 <= covariance[i] <= limit
            for i, limit in ((0, xy_max), (7, xy_max), (35, yaw_max))):
        reasons.append('covariance_invalid')
    if not quality.get('ok'):
        reasons.append('registration_not_ok')
    if not quality.get('converged'):
        reasons.append('not_converged')
    fitness = float(quality.get('fitness', float('nan')))
    inlier = float(quality.get('inlier_ratio', float('nan')))
    count = int(quality.get('correspondences', 0))
    if not math.isfinite(fitness) or fitness > fitness_max:
        reasons.append('fitness_exceeded')
    if not math.isfinite(inlier) or inlier < inlier_min:
        reasons.append('inlier_insufficient')
    if count < correspondences_min:
        reasons.append('correspondences_insufficient')
    return {
        'failure_reasons': ','.join(reasons), 'converged': bool(quality.get('converged')),
        'fitness': fitness, 'fitness_max': fitness_max, 'inlier_ratio': inlier,
        'inlier_min': inlier_min, 'correspondences': count, 'correspondences_min': correspondences_min,
        'xy_variance_max': xy_max, 'yaw_variance_max': yaw_max,
        'sample_stamp': quality.get('stamp', 0.0),
    }
