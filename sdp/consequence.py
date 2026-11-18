"""Consequence analysis (DP-2): онлайн-оценка последствий единичного отказа.

Аналог функции Consequence Analysis на промышленных DP-пультах
(Kongsberg, GE, Wärtsilä): система непрерывно отвечает на вопрос
«если прямо сейчас откажет ещё один агрегат — удержим ли позицию?».

Два показателя:
  1. check()             — осуществимость и загрузка движителей после
                           отказа каждого из оставшихся (с запасом margin)
  2. survivable_wind()   — предельный ветер в текущем направлении среды
                           после худшего из оставшихся единичных отказов
"""
import numpy as np

from .allocation import AzimuthAllocator

# Допуски осуществимости (как в capability.py)
F_TOL = 1.0e3
N_TOL = 3.0e4


class ConsequenceAnalyzer:
    def __init__(self, thrusters, margin=0.1):
        self.thrusters = thrusters
        self.margin = margin          # запас к текущей нагрузке (10%)

    def _active(self):
        return [i for i, th in enumerate(self.thrusters) if not th.failed]

    @staticmethod
    def _feasible(alloc, tau, active):
        f = alloc._constrained_forces(-tau, active)
        res = -tau - alloc._generalized(f)
        return np.hypot(res[0], res[1]) < F_TOL and abs(res[2]) < N_TOL, f

    def check(self, tau_load_body):
        """Последствия отказа каждого оставшегося движителя.

        tau_load_body — текущая уравновешиваемая нагрузка среды (body-кадр).
        Возвращает {idx: (feasible, utilization)}.
        """
        tau = np.asarray(tau_load_body, float) * (1.0 + self.margin)
        results = {}
        for j in self._active():
            active = [i for i in self._active() if i != j]
            alloc = AzimuthAllocator(self.thrusters)
            ok, f = self._feasible(alloc, tau, active)
            util = max((np.hypot(*f[i]) / self.thrusters[i].t_max
                        for i in active), default=1.0)
            results[j] = (ok, float(util))
        return results

    def survivable_wind(self, beta_deg, hs, v_hi=60.0, n_iter=18):
        """Предельный ветер после худшего из оставшихся единичных отказов.

        Направление среды beta_deg и Hs — текущие (body-кадр, psi ≈ 0).
        """
        from capability import env_load          # во избежание дублирования
        worst = v_hi
        for j in self._active():
            active = [i for i in self._active() if i != j]
            alloc = AzimuthAllocator(self.thrusters)

            def feas(v):
                tau = env_load(beta_deg, v, hs)
                ok, _ = self._feasible(alloc, tau, active)
                return ok

            lo, hi = 0.0, v_hi
            if feas(hi):
                lo = hi
            else:
                for _ in range(n_iter):
                    mid = 0.5 * (lo + hi)
                    if feas(mid):
                        lo = mid
                    else:
                        hi = mid
            worst = min(worst, lo)
        return worst
