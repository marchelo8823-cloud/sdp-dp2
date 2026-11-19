"""Онлайн-диаграмма возможностей (capability) для живого пульта.

В отличие от capability.py (офлайн-анализ с эталонной конфигурацией),
здесь поляра строится по ТЕКУЩЕМУ состоянию движителей контура:
учитываются отказы, введённые инструктором/аварией, и текущая Hs.

Две кривые (как K-Pos №10 / MT NavDP №6):
  intact — все исправные на данный момент движители;
  worst  — минимум по ещё одному одиночному отказу (критерий DP-2
           «после следующего отказа удержание сохраняется»).

Углы beta — направление среды ОТНОСИТЕЛЬНО НОСА (0° = ветер в нос),
скорости — м/с; на пульте переводятся в узлы и поворачиваются на курс.
"""
import numpy as np

from capability import env_load, feasible, max_wind
from sdp.allocation import AzimuthAllocator, AzimuthThruster

KN = 1.94384


def mirror_allocator(thrusters, failed_extra=None):
    """Копия конфигурации движителей контура для статического анализа.

    Копии нужны, чтобы пробы отказов не трогали живые объекты контура.
    """
    lst = []
    for i, th in enumerate(thrusters):
        t = AzimuthThruster(lx=th.lx, ly=th.ly, t_max=th.t_max,
                            tunnel=getattr(th, "tunnel", False))
        t.failed = bool(th.failed) or (i == failed_extra)
        lst.append(t)
    return AzimuthAllocator(lst)


def live_polar(thrusters, hs, n_beta=36, v_hi=60.0):
    """Поляра предельного ветра по текущей конфигурации.

    Возвращает (betas_deg, intact_ms, worst_ms).
    """
    betas = np.linspace(0.0, 360.0, n_beta, endpoint=False)
    alloc_ok = mirror_allocator(thrusters)
    intact = np.array([max_wind(b, alloc_ok, hs, v_hi=v_hi) for b in betas])
    worst = intact.copy()
    for i, th in enumerate(thrusters):
        if th.failed:
            continue
        alloc_f = mirror_allocator(thrusters, failed_extra=i)
        v = np.array([max_wind(b, alloc_f, hs, v_hi=v_hi) for b in betas])
        worst = np.minimum(worst, v)
    return betas, intact, worst


def margin_at(thrusters, hs, wind_dir_rel_deg, wind_speed_ms):
    """Запас по ветру при текущем направлении среды (критерий DP-2).

    Возвращает (предел_мс, запас_мс): предельная скорость по кривой
    worst при данном beta и сколько осталось до неё.
    """
    betas, _, worst = live_polar(thrusters, hs, n_beta=36)
    k = int(round(wind_dir_rel_deg % 360.0 / 10.0)) % len(betas)
    lim = float(worst[k])
    return lim, lim - wind_speed_ms
