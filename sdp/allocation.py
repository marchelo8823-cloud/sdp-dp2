"""Распределение тяги по движительно-рулевым установкам.

Два уровня:

1. ThrustAllocator — базовый: азимуты фиксированы, силы по
   псевдообратной матрице с насыщением.

2. AzimuthAllocator — динамический: азимуты и тяга решаются совместно.
   Параметризация векторами сил f_i = (fx_i, fy_i) делает задачу
   ЛИНЕЙНОЙ:  tau = A @ f,  где для i-го движителя
       A[:, i] = [[1, 0], [0, 1], [-ly_i, lx_i]]
   Решение — QP с взвешенной минимизацией мощности (KKT), затем
   наложение физических ограничений:
     - |f_i| <= t_max            (насыщение, с итеративной перераздачей)
     - |d(alpha)/dt| <= rate     (скорость поворота азимута)
     - запрещённые зоны азимута  (гистерезис по ближайшей кромке)
   Отказавший движитель исключается из задачи — переконфигурация DP-2.
"""
import numpy as np


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# Базовый аллокатор (фиксированные азимуты)
# ---------------------------------------------------------------------------

class Thruster:
    def __init__(self, lx, ly, azimuth_deg, t_max=2.5e5):
        self.lx, self.ly = lx, ly
        self.azimuth = np.deg2rad(azimuth_deg)
        self.t_max = t_max
        self.failed = False   # для сценариев DP-2 single failure


class ThrustAllocator:
    """B(alpha) @ u = tau,  B[:, i] = [cos a, sin a, lx sin a - ly cos a]."""

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


# ---------------------------------------------------------------------------
# Динамический азимутальный аллокатор (QP + ограничения)
# ---------------------------------------------------------------------------

class AzimuthThruster:
    """Азимутальная ДРУ с физическими ограничениями.

    tunnel=True  — туннельный движитель: азимут жёстко фиксирован,
                   тяга знакопеременная (реверс шага).
    forbidden_zones — кортежи (start_deg, end_deg), start < end,
                   в диапазоне (-180, 180].
    """
    def __init__(self, lx, ly, t_max=2.5e5, az0_deg=0.0,
                 az_rate_deg_s=10.0, forbidden_zones=(), tunnel=False):
        self.lx, self.ly = lx, ly
        self.t_max = t_max
        self.az_rate = np.deg2rad(az_rate_deg_s)
        self.forbidden_zones = [tuple(np.deg2rad(z)) for z in forbidden_zones]
        self.tunnel = tunnel
        self.failed = False
        # Текущее состояние
        self.alpha = np.deg2rad(az0_deg)
        self.u = 0.0

    def in_forbidden(self, alpha):
        for a, b in self.forbidden_zones:
            if a <= alpha <= b:
                return (a, b)
        return None


class AzimuthAllocator:
    def __init__(self, thrusters, reg=1e-6):
        self.thrusters = thrusters
        self.reg = reg

    # --- QP-ядро ---------------------------------------------------------
    def _solve_forces(self, tau, active):
        """min sum |f_i|²/t_max_i²  s.t.  A f = tau  (KKT-система)."""
        m = len(active)
        A = np.zeros((3, 2 * m))
        Winv = np.zeros(2 * m)
        for j, i in enumerate(active):
            th = self.thrusters[i]
            A[0, 2 * j] = 1.0
            A[1, 2 * j + 1] = 1.0
            A[2, 2 * j] = -th.ly
            A[2, 2 * j + 1] = th.lx
            Winv[2 * j] = Winv[2 * j + 1] = th.t_max**2
        M = A @ (A * Winv[None, :]).T
        # Относительная регуляризация + pinv: устойчиво при ранге A < 3
        # (например, когда после насыщения свободен один движитель)
        scale = max(1.0, float(np.abs(M).max()))
        lam = np.linalg.pinv(M + self.reg * scale * np.eye(3)) @ tau
        f = (A * Winv[None, :]).T @ lam
        return f.reshape(m, 2)

    def _constrained_forces(self, tau, active):
        """QP + насыщение: итеративная перераздача после клиппинга."""
        f = np.zeros((len(self.thrusters), 2))
        if not active:
            return f
        free = list(active)
        tau_res = np.asarray(tau, float).copy()
        for _ in range(3):
            if not free or np.linalg.norm(tau_res) < 1e-9:
                break
            sol = self._solve_forces(tau_res, free)
            clipped = []
            for j, i in enumerate(free):
                th = self.thrusters[i]
                mag = np.hypot(*sol[j])
                if mag > th.t_max:
                    sol[j] *= th.t_max / mag
                    clipped.append(i)
                f[i] += sol[j]
            # Достигнутое клиппированными — вычитаем из остатка
            tau_res = np.asarray(tau, float) - self._generalized(f)
            free = [i for i in free if i not in clipped]
        return f

    def _generalized(self, f):
        """Вектор сил (N,2) -> обобщённые силы tau (3)."""
        tau = np.zeros(3)
        for i, th in enumerate(self.thrusters):
            fx, fy = f[i]
            tau += np.array([fx, fy, th.lx * fy - th.ly * fx])
        return tau

    # --- Ограничения азимута ----------------------------------------------
    def _apply_azimuth_limits(self, th, f_vec, dt):
        mag = np.hypot(*f_vec)
        # Мёртвая зона: при малом запросе тягу снимаем, азимут держим —
        # исключаем «охоту» азимута около нулевой тяги
        if mag < max(1e-6, 0.02 * th.t_max):
            return 0.0, th.alpha
        a_des = np.arctan2(f_vec[1], f_vec[0])
        # Два представления: (a, +u) и (a+pi, -u); переключаемся только
        # с гистерезисом 20° — против дребезга реверса
        a_alt = wrap_pi(a_des + np.pi)
        d_cur = abs(wrap_pi(a_des - th.alpha))
        d_alt = abs(wrap_pi(a_alt - th.alpha))
        if d_alt < d_cur - np.deg2rad(20.0):
            a_des, u_des = a_alt, -mag
        else:
            u_des = mag
        # Запрещённая зона: к ближайшей кромке от текущего положения
        zone = th.in_forbidden(a_des)
        if zone is not None:
            a, b = zone
            a_des = a if abs(wrap_pi(th.alpha - a)) < abs(wrap_pi(th.alpha - b)) else b
            u_des = mag * np.cos(wrap_pi(np.arctan2(f_vec[1], f_vec[0]) - a_des))
        if th.tunnel:
            a_des = th.alpha                       # азимут не меняется
            u_des = f_vec[0] * np.cos(th.alpha) + f_vec[1] * np.sin(th.alpha)
        # Скорость поворота
        da = wrap_pi(a_des - th.alpha)
        da = np.clip(da, -th.az_rate * dt, th.az_rate * dt)
        alpha_new = wrap_pi(th.alpha + da)
        # Тяга с учётом фактического угла (проекция запрошенной силы)
        u_new = np.clip(u_des, -th.t_max, th.t_max)
        return u_new, alpha_new

    # --- Основной вызов -----------------------------------------------------
    def allocate(self, tau, dt):
        """Возвращает (u[N], alpha[N], достигнутый tau)."""
        active = [i for i, th in enumerate(self.thrusters) if not th.failed]
        f = self._constrained_forces(tau, active)
        for i, th in enumerate(self.thrusters):
            if th.failed:
                th.u, _ = 0.0, th.alpha
                continue
            th.u, th.alpha = self._apply_azimuth_limits(th, f[i], dt)
        f_act = np.array([[th.u * np.cos(th.alpha), th.u * np.sin(th.alpha)]
                          if not th.failed else [0.0, 0.0]
                          for th in self.thrusters])
        u = np.array([th.u for th in self.thrusters])
        alpha = np.array([th.alpha for th in self.thrusters])
        return u, alpha, self._generalized(f_act)
