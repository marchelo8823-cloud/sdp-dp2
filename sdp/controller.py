"""PID-регулятор DP с ветровой компенсацией (feedforward).

tau = R(psi)^T @ (Kp @ e + Ki @ int(e) - Kd @ R(psi) @ nu) - tau_wind_ff
e = eta_d - eta (NED)
"""
import numpy as np
from .vessel import Vessel3DOF


class PIDController:
    def __init__(self,
                 Kp=(2.0e5, 2.0e5, 2.0e7),
                 Ki=(2.0e3, 2.0e3, 2.0e5),
                 Kd=(1.8e6, 3.5e6, 1.6e8),
                 i_limit=3.0e5):
        self.Kp = np.diag(Kp)
        self.Ki = np.diag(Ki)
        self.Kd = np.diag(Kd)
        self.i_limit = i_limit
        self.integral = np.zeros(3)

    def reset(self):
        self.integral[:] = 0.0

    def compute(self, eta_d, eta_hat, nu_hat, tau_wind_ff, dt):
        e = np.asarray(eta_d, float) - np.asarray(eta_hat, float)
        e[2] = (e[2] + np.pi) % (2 * np.pi) - np.pi
        self.integral += e * dt
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        psi = eta_hat[2]
        R = Vessel3DOF.R(psi)
        e_dot = R @ np.asarray(nu_hat, float)  # НЧ-скорость в NED

        tau_ned = (self.Kp @ e + self.Ki @ self.integral
                   - self.Kd @ e_dot - np.asarray(tau_wind_ff, float))
        return R.T @ tau_ned   # в body-кадр
