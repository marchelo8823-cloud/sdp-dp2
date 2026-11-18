"""Распределение тяги по движительно-рулевым установкам (азимутальным).

Базовый вариант: азимуты фиксированы, силы — по псевдообратной матрице
с насыщением. Развитие (DP-2): QP-оптимизация с запрещёнными зонами,
учёт redundancy groups и отказов.

B(alpha) @ u = tau,  где для i-го движителя:
  B[:, i] = [cos(a_i), sin(a_i), lx_i*sin(a_i) - ly_i*cos(a_i)]

ВАЖНО: конфигурация движителей должна давать матрицу B полного ранга 3.
Например: носовой туннельный (аз. 90°) + 2 кормовых азимутальных (±60°).
"""
import numpy as np


class Thruster:
    def __init__(self, lx, ly, azimuth_deg, t_max=2.5e5):
        self.lx, self.ly = lx, ly
        self.azimuth = np.deg2rad(azimuth_deg)
        self.t_max = t_max
        self.failed = False   # для сценариев DP-2 single failure


class ThrustAllocator:
    def __init__(self, thrusters):
        self.thrusters = thrusters

    def _B(self):
        cols = []
        for th in self.thrusters:
            if th.failed:
                cols.append(np.zeros(3))
                continue
            a = th.azimuth
            cols.append(np.array([np.cos(a), np.sin(a),
                                  th.lx * np.sin(a) - th.ly * np.cos(a)]))
        return np.array(cols).T   # 3 x N

    def allocate(self, tau):
        """Возвращает (вектор сил по движителям, достигнутый tau)."""
        B = self._B()
        u = np.linalg.pinv(B) @ np.asarray(tau, float)
        for i, th in enumerate(self.thrusters):
            if th.failed:
                u[i] = 0.0
            else:
                u[i] = np.clip(u[i], -th.t_max, th.t_max)
        return u, B @ u
