"""Наблюдатель СДП: расширенный фильтр Калмана.

Состояния (14):
  xi_w[0:4]  — волновые (WF) состояния по x и y: [h_x, h_x', h_y, h_y']
  eta[4:7]   — НЧ положение [x, y, psi]  (NED)
  nu[7:10]   — скорости body [u, v, r]
  b[10:13]   — медленно меняющееся смещение сил (ветер/течение/модель), NED

Измерение: z = eta + C_w @ xi_w + шум
"""
import numpy as np
from .vessel import Vessel3DOF


class DPObserver:
    def __init__(self, vessel: Vessel3DOF, dt=0.1,
                 omega0=0.9, zeta=0.08,
                 q_wave=1.0, q_bias=(1e2, 1e2, 1e3),
                 r_meas=(0.5**2, 0.5**2, np.deg2rad(0.3)**2)):
        self.v = vessel
        self.dt = dt
        self.n = 13
        # Волновая модель 2-го порядка (относительная формулировка)
        # h'' + 2*zeta*w0*h' + w0^2*h = w
        A1 = np.array([[0.0, 1.0],
                       [-omega0**2, -2 * zeta * omega0]])
        self.A_w = np.zeros((4, 4))
        self.A_w[0:2, 0:2] = A1
        self.A_w[2:4, 2:4] = A1

        # Непрерывный шум процесса
        self.Qc = np.diag([q_wave, q_wave, q_wave, q_wave,   # WF-возбуждение
                           1e-4, 1e-4, 1e-6,                  # eta
                           1e-3, 1e-3, 1e-5,                  # nu
                           *q_bias])                          # bias random walk
        self.R = np.diag(r_meas)

        self.x = np.zeros(self.n)
        self.P = np.eye(self.n) * 10.0
        # Калмановская коррекция состояния за последний такт
        # (нужна адаптивному наблюдателю возмущений)
        self.last_dx = np.zeros(self.n)

    def _f(self, x, tau):
        """Непрерывная динамика f(x, tau)."""
        xi, eta, nu, b = x[0:4], x[4:7], x[7:10], x[10:13]
        R = Vessel3DOF.R(eta[2])
        dx = np.zeros(self.n)
        dx[0:4] = self.A_w @ xi
        dx[4:7] = R @ nu
        # tau — в body-кадре, b — в NED
        dx[7:10] = self.v.Minv @ (R.T @ b + np.asarray(tau, float)
                                  - self.v.D @ nu)
        dx[10:13] = 0.0
        return dx

    def _jacobian(self, x, tau):
        """Якобиан df/dx в точке x."""
        xi, eta, nu, b = x[0:4], x[4:7], x[7:10], x[10:13]
        psi = eta[2]
        R = Vessel3DOF.R(psi)
        u, v, _ = nu
        c, s = np.cos(psi), np.sin(psi)

        F = np.zeros((self.n, self.n))
        F[0:4, 0:4] = self.A_w
        # d(eta_dot)/dnu и d(eta_dot)/dpsi
        F[4:7, 7:10] = R
        F[4, 6] = -u * s - v * c
        F[5, 6] = u * c - v * s
        # d(nu_dot)/dpsi, /dnu, /db (tau — body, b — NED)
        dR_T = np.array([[-s, c, 0.0], [-c, -s, 0.0], [0.0, 0.0, 0.0]])
        F[7:10, 6] = self.v.Minv @ (dR_T @ b)
        F[7:10, 7:10] = -self.v.Minv @ self.v.D
        F[7:10, 10:13] = self.v.Minv @ R.T
        return F

    def step(self, z, tau):
        """Предикция + коррекция. z — измерение [x, y, psi]."""
        dt = self.dt
        # Предикция состояния (Эйлер)
        self.x = self.x + self._f(self.x, tau) * dt
        # Предикция ковариации
        F = self._jacobian(self.x, tau)
        Fd = np.eye(self.n) + F * dt
        Qd = self.Qc * dt
        self.P = Fd @ self.P @ Fd.T + Qd
        # Коррекция
        H = np.zeros((3, self.n))
        H[0, 0] = 1.0   # WF-составляющая x
        H[1, 2] = 1.0   # WF-составляющая y
        H[0:3, 4:7] = np.eye(3)
        z = np.asarray(z, float)
        # Нормализация угловой инновации
        innov = z - H @ self.x
        innov[2] = (innov[2] + np.pi) % (2 * np.pi) - np.pi
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.last_dx = K @ innov
        self.P = (np.eye(self.n) - K @ H) @ self.P
        self.clamp_bias()

    def clamp_bias(self, lim=(1.5e5, 1.5e5, 4.0e5)):
        """Ограничение оценки смещения (anti-windup).

        Bias — медленная составляющая силы среды (ветер+течение+ошибка
        модели), для судна такого класса это десятки кН. Без насыщения
        random-walk состояние b в долгих ручных манёврах уходит за
        1e5 кН: consequence-анализ принимает его за чудовищную среду
        и объявляет любой отказ непереносимым."""
        lim = np.asarray(lim, float)
        self.x[10:13] = np.clip(self.x[10:13], -lim, lim)

    @property
    def eta_hat(self):
        return self.x[4:7]

    @property
    def bias(self):
        return self.x[10:13]

    @property
    def nu_hat(self):
        return self.x[7:10]

    @property
    def bias_hat(self):
        return self.x[10:13]
