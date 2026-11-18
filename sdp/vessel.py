"""3-DOF модель судна (surge, sway, yaw) для DP-симуляции.

Параметры соответствуют типичному offshore supply vessel (Fossen, 2021).
Кинематика: eta_dot = R(psi) @ nu
Динамика:   M @ nu_dot + C(nu) @ nu + D @ nu = tau + tau_env
"""
import numpy as np


class Vessel3DOF:
    def __init__(self):
        self.L = 76.2            # длина, м
        self.m = 4.0e6           # масса, кг
        self.Iz = 3.0e8          # момент инерции по yaw, кг·м²

        # Присоединённые массы
        Xudot = -0.05 * self.m
        Yvdot = -0.50 * self.m
        Nrdot = -0.08 * self.Iz
        self.M = np.diag([self.m - Xudot,
                          self.m - Yvdot,
                          self.Iz - Nrdot])
        self.Minv = np.linalg.inv(self.M)

        # Линейное вязкое демпфирование
        self.D = np.diag([5.0e4, 1.5e5, 1.5e7])

    @staticmethod
    def R(psi):
        """Матрица поворота body->NED (3DOF)."""
        c, s = np.cos(psi), np.sin(psi)
        return np.array([[c, -s, 0.0],
                         [s,  c, 0.0],
                         [0.0, 0.0, 1.0]])

    def coriolis(self, nu):
        """Приближённый член Кориолиса/центробежных сил."""
        u, v, r = nu
        return self.m * np.array([v * r, -u * r, 0.0])

    def step(self, eta, nu, tau, tau_env, dt):
        """Один шаг интегрирования (Эйлер). eta=[x,y,psi], nu=[u,v,r]."""
        nu_dot = self.Minv @ (np.asarray(tau, float) + np.asarray(tau_env, float)
                              - self.D @ nu - self.coriolis(nu))
        nu_new = nu + nu_dot * dt
        eta_new = eta + self.R(eta[2]) @ nu_new * dt
        return eta_new, nu_new
