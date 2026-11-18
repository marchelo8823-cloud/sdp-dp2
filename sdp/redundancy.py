"""Резервирование датчиков (DP-2): вотирование и инъекция отказов.

- vote_median: медианное вотирование по 3 независимым измерениям
- SensorSuite: набор PRS/гирокомпасов с флагами отказов
"""
import numpy as np


def vote_median(measurements):
    """Медианное вотирование по набору измерений (N, 3)."""
    m = np.asarray(measurements, float)
    return np.median(m, axis=0)


class SensorSuite:
    """Три PRS + два гирокомпаса; поддерживает отказы (single failure)."""

    def __init__(self, prs_noise=0.3, gyro_noise_deg=0.1, seed=1):
        self.rng = np.random.default_rng(seed)
        self.prs_failed = [False, False, False]
        self.gyro_failed = [False, False]
        self.prs_noise = prs_noise
        self.gyro_noise = np.deg2rad(gyro_noise_deg)

    def fail_prs(self, idx):
        self.prs_failed[idx] = True

    def read_position(self, eta_true):
        """Возвращает свотированное измерение [x, y] или None при недоступности."""
        readings = []
        for i, failed in enumerate(self.prs_failed):
            if not failed:
                readings.append(eta_true[0:2]
                                + self.rng.normal(0, self.prs_noise, 2))
        if len(readings) < 2:
            return None     # потеря PRS-покрытия
        return np.median(np.array(readings), axis=0)

    def read_heading(self, psi_true):
        readings = [psi_true + self.rng.normal(0, self.gyro_noise)
                    for failed in self.gyro_failed if not failed]
        if not readings:
            return None
        return float(np.mean(readings))
