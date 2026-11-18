"""Адаптивный наблюдатель возмущений (RBF-сеть, онлайн-обучение).

Идея (по мотивам ELM/RBF disturbance observer из литературы 2023–2025):
медленный bias в EKF заменяется двухканальной схемой —

    d_true(body) ≈ Wᵀ φ(ν̂)  +  b_residual(EKF)

RBF-сеть W ᵀφ учит СТРУКТУРИРОВАННУЮ часть возмущения (квадратичные
нагрузки, дрейф 2-го порядка, рассогласование модели) как функцию
НЧ-скорости; EKF отслеживает только необъяснённый остаток.

Сигнал адаптации — калмановская коррекция bias-состояния за такт
(та часть возмущения, которую фильтру пришлось «догонять»). Если сеть
объясняет возмущение, коррекция → 0. Нормированный LMS с мёртвой зоной,
утечкой и проекцией весов — гарантированная ограниченность.
"""
import numpy as np


class RBFDisturbanceObserver:
    """Δd = Wᵀ φ(ν̂) в body-кадре, онлайн-адаптация по инновации EKF."""

    def __init__(self, centers_uv=(-1.0, -0.5, 0.0, 0.5, 1.0),
                 centers_r=(-0.1, 0.0, 0.1),
                 sigma=0.6, mu=0.3, leak=1e-6,
                 dead=0.1, w_max=5.0e5):
        cs = [(u, v, r) for u in centers_uv for v in centers_uv
              for r in centers_r]
        self.c = np.asarray(cs, float)          # (N, 3)
        self.sigma = float(sigma)
        self.mu = float(mu)
        self.leak = float(leak)
        self.dead = float(dead)                 # мёртвая зона, Н
        self.w_max = float(w_max)
        self.W = np.zeros((len(cs), 3))

    def phi(self, nu):
        d2 = ((self.c - np.asarray(nu, float)) ** 2).sum(axis=1)
        return np.exp(-d2 / (2.0 * self.sigma ** 2))

    def estimate(self, nu):
        """Структурированная часть возмущения, body-кадр [Н, Н, Н·м]."""
        return self.phi(nu) @ self.W

    def adapt(self, nu, err_body, dt):
        """Шаг адаптации. err_body — коррекция bias EKF за такт (body).

        Коррекция — это ПРИРАЩЕНИЕ оценки возмущения за такт, поэтому
        LMS-член без dt (интегрирование приращений воспроизводит сигнал);
        утечка — со временем.
        """
        p = self.phi(nu)
        e = np.asarray(err_body, float)
        if np.linalg.norm(e) < self.dead:
            e = np.zeros(3)
        self.W += (self.mu * np.outer(p, e) / (1.0 + p @ p)
                   - dt * self.leak * self.W)
        np.clip(self.W, -self.w_max, self.w_max, out=self.W)
