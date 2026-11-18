"""Модель среды: волнение (JONSWAP), ветер, течение."""
import numpy as np

G = 9.81


def jonswap_spectrum(omega, hs, tp, gamma=3.3):
    """Спектр JONSWAP, м²·с. omega — рад/с."""
    wp = 2.0 * np.pi / tp
    sigma = np.where(omega <= wp, 0.07, 0.09)
    alpha = 0.0081
    s = (alpha * G**2 / omega**5
         * np.exp(-1.25 * (wp / omega)**4)
         * gamma ** np.exp(-0.5 * ((omega - wp) / (sigma * wp))**2))
    return np.where(omega > 0, s, 0.0)


class WaveForce:
    """Волновые силы первого порядка как сумма гармоник по спектру JONSWAP.

    force_scale калибрует амплитуду отклика (эмпирический RAO-эквивалент).
    """
    def __init__(self, hs=2.0, tp=7.0, heading_deg=45.0, n_comp=40,
                 force_scale=3.0e4, seed=42):
        rng = np.random.default_rng(seed)
        w = np.linspace(0.3, 2.0, n_comp)
        dw = w[1] - w[0]
        s = jonswap_spectrum(w, hs, tp)
        self.w = w
        self.amp = np.sqrt(2.0 * s * dw) * force_scale
        self.phase = rng.uniform(0, 2 * np.pi, n_comp)
        beta = np.deg2rad(heading_deg)
        # Проекции волновой силы на surge/sway/yaw
        self.dir = np.array([np.cos(beta), np.sin(beta), 0.3 * np.sin(beta)])

    def force(self, t):
        osc = np.sum(self.amp * np.sin(self.w * t + self.phase))
        return self.dir * osc


class Wind:
    """Средняя ветровая нагрузка (Сёренсен): tau = 0.5*rho*V²*A*C(gamma)."""
    def __init__(self, speed=10.0, dir_deg=30.0,
                 Ax=120.0, Ay=600.0, AL=60.0, Cx=0.5, Cy=0.8, Cn=0.4):
        self.rho = 1.225
        self.V = speed
        self.gamma = np.deg2rad(dir_deg)
        self.Ax, self.Ay, self.AL = Ax, Ay, AL
        self.Cx, self.Cy, self.Cn = Cx, Cy, Cn

    def force(self):
        q = 0.5 * self.rho * self.V**2
        return np.array([
            q * self.Ax * self.Cx * np.cos(self.gamma),
            q * self.Ay * self.Cy * np.sin(self.gamma),
            q * self.Ay * self.AL * self.Cn * np.sin(self.gamma),
        ])


class Current:
    """Постоянное течение (в кадре NED), действует через демпфирование."""
    def __init__(self, speed=0.3, dir_deg=180.0):
        b = np.deg2rad(dir_deg)
        self.vc = np.array([speed * np.cos(b), speed * np.sin(b)])
