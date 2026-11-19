"""Резервирование датчиков (DP-2): отбраковка, веса, вотирование.

Каждая система положения (PRS) и каждый гирокомпас проходят
онлайн-проверки качества данных:

- «замирание»    — дисперсия окна ниже порога (датчик завис);
- «разброс»      — шум выше допустимого;
- «скачок»       — мгновенное изменение сверх лимита;
- «расхождение»  — устойчивый уход от медианы остальных систем.

Отбракованная система исключается из вотирования до явного возврата
оператором. Веса в вотировании пропорциональны 1/σ² (σ — онлайн-оценка
СКО канала). Инструктор может вводить нештатные режимы:
``fail`` (нет данных), ``frozen`` (замирание), ``noisy`` (разброс),
``drift`` (медленный уход — «плывущая» система).
"""
from collections import deque

import numpy as np

WIN = 100            # окно тестов, отсчётов (10 с при 10 Гц)
FROZEN_EPS = 0.05    # замирание: размах окна меньше этого, м (или °)
NOISE_K = 3.0        # разброс: σ окна больше K·номинала
JUMP_LIM = 3.0       # скачок за один такт, м (или °)
DEV_LIM = 2.5        # расхождение с медианой, м
DEV_TIME = 5.0       # …устойчивое столько секунд
GYRO_DISAGREE_DEG = 2.0   # расхождение двух гирокомпасов, °
GYRO_DISAGREE_T = 3.0     # …устойчивое, с
GYRO_JUMP_DEG = 5.0       # скачок курса за такт 0.1 с, ° (выше любого
                          # разворота DP-судна — только неисправность)
GYRO_FROZEN_EPS_DEG = 0.02  # замирание гирокомпаса, °


def vote_median(measurements):
    """Медианное вотирование по набору измерений (N, 3)."""
    m = np.asarray(measurements, float)
    return np.median(m, axis=0)


class PrsChannel:
    """Одна система положения: состояние, тесты, вес."""

    def __init__(self, name, noise=0.3):
        self.name = name
        self.noise = noise
        self.failed = False       # нет данных (отказ агрегата)
        self.deselected = False   # снята оператором
        self.rejected = False     # отбракована автоматикой
        self.reason = ""          # причина отбраковки
        self.fault = None         # имитированный сбой (frozen/noisy/drift)
        self.sigma = noise        # онлайн-оценка СКО, м
        self.weight = 0.0         # доля в вотировании, 0..1
        self.hist = deque(maxlen=WIN)
        self._frozen_val = None
        self._drift_t0 = None
        self._dev_since = None
        self.disagree = False     # расходится с другой системой (2 в работе)

    @property
    def active(self):
        return not (self.failed or self.deselected or self.rejected)

    @property
    def state(self):
        if self.failed:
            return "failed"
        if self.deselected:
            return "deselected"
        if self.rejected:
            return "rejected"
        return "ok"

    def clear_fault(self):
        self.failed = False
        self.fault = None
        self._frozen_val = None
        self._drift_t0 = None

    def restore(self):
        """Возврат оператором: снять отбраковку/снятие, обнулить окно."""
        self.rejected = False
        self.deselected = False
        self.reason = ""
        self.hist.clear()
        self._dev_since = None
        self.sigma = self.noise

    def read(self, xy, t, rng):
        """Сырое измерение с учётом имитированного сбоя; None, если нет данных."""
        if self.failed:
            return None
        if self.fault == "frozen":
            if self._frozen_val is None:
                self._frozen_val = np.array(xy, float) \
                    + rng.normal(0, self.noise, 2)
            r = self._frozen_val.copy()
        elif self.fault == "noisy":
            r = np.array(xy, float) + rng.normal(0, self.noise * 6, 2)
        elif self.fault == "drift":
            if self._drift_t0 is None:
                self._drift_t0 = t
            # медленный уход 0.5 м/с по диагонали — «плывущая» система
            d = 0.5 * (t - self._drift_t0)
            r = np.array(xy, float) + np.array([d, d * 0.6]) \
                + rng.normal(0, self.noise, 2)
        else:
            r = np.array(xy, float) + rng.normal(0, self.noise, 2)
        self.hist.append((t, r))
        return r

    def test(self, t, others_med):
        """Проверки качества; возвращает причину отбраковки или None.

        Замирание и разброс считаются по первым разностям — тренд
        (движение судна) не даёт ложных срабатываний.
        """
        h = self.hist
        if len(h) >= 5:
            prev = h[-2][1]
            cur = h[-1][1]
            if np.hypot(*(cur - prev)) > JUMP_LIM:
                return "скачок"
        if len(h) >= WIN // 2:
            arr = np.array([p[1] for p in list(h)[-WIN // 2:]])
            dif = np.diff(arr, axis=0)
            if np.linalg.norm(dif, axis=1).max() < FROZEN_EPS:
                return "замирание"
            if float(dif.std()) > NOISE_K * self.noise * np.sqrt(2):
                return "разброс"
        if others_med is not None and len(h) >= 5:
            dev = float(np.hypot(*(h[-1][1] - others_med)))
            if dev > DEV_LIM:
                if self._dev_since is None:
                    self._dev_since = t
                elif t - self._dev_since >= DEV_TIME:
                    return f"расхождение с медианой {dev:.1f} м"
            else:
                self._dev_since = None
        return None


class GyroChannel:
    """Один гирокомпас: состояние и тесты (замирание/разброс/скачок)."""

    def __init__(self, name, noise_rad):
        self.name = name
        self.noise = noise_rad
        self.failed = False
        self.deselected = False
        self.rejected = False
        self.reason = ""
        self.fault = None
        self.disagree = False     # расходится с другим гирокомпасом
        self.used = False         # входит в принятый курс
        self.dev_deg = 0.0        # отклонение от принятого курса, °
        self.hist = deque(maxlen=WIN)
        self._frozen_val = None

    @property
    def active(self):
        return not (self.failed or self.deselected or self.rejected)

    @property
    def state(self):
        if self.failed:
            return "failed"
        if self.deselected:
            return "deselected"
        if self.rejected:
            return "rejected"
        return "ok"

    def clear_fault(self):
        self.failed = False
        self.fault = None
        self._frozen_val = None

    def restore(self):
        self.rejected = False
        self.deselected = False
        self.reason = ""
        self.hist.clear()

    def read(self, psi, t, rng):
        if self.failed:
            return None
        if self.fault == "frozen":
            if self._frozen_val is None:
                self._frozen_val = psi + rng.normal(0, self.noise)
            r = self._frozen_val
        elif self.fault == "noisy":
            r = psi + rng.normal(0, self.noise * 6)
        else:
            r = psi + rng.normal(0, self.noise)
        self.hist.append((t, r))
        return r

    def test(self):
        """Тесты по первым разностям (поворот судна — не сбой)."""
        h = self.hist
        if len(h) >= 5:
            if abs(h[-1][1] - h[-2][1]) > np.deg2rad(GYRO_JUMP_DEG):
                return "скачок"
        if len(h) >= WIN // 2:
            arr = np.array([p[1] for p in list(h)[-WIN // 2:]])
            dif = np.diff(arr)
            if float(np.abs(dif).max()) < np.deg2rad(GYRO_FROZEN_EPS_DEG):
                return "замирание"
            if float(dif.std()) > NOISE_K * self.noise * np.sqrt(2):
                return "разброс"
        return None


class SensorSuite:
    """Три PRS + два гирокомпаса: отбраковка, веса, вотирование."""

    def __init__(self, prs_noise=0.3, gyro_noise_deg=0.1, seed=1):
        self.rng = np.random.default_rng(seed)
        self.prs = [PrsChannel(f"PRS {i + 1}", prs_noise) for i in range(3)]
        self.gyro = [GyroChannel(f"ГИРО {i + 1}",
                                 np.deg2rad(gyro_noise_deg)) for i in range(2)]
        self._last_est = None     # последняя свотированная позиция

    # --------------------------------------------------- совместимость
    @property
    def prs_failed(self):
        return [ch.failed for ch in self.prs]

    @property
    def gyro_failed(self):
        return [ch.failed for ch in self.gyro]

    # --------------------------------------------------- инъекции
    def fail_prs(self, idx, ftype="fail"):
        ch = self.prs[idx]
        ch.clear_fault()
        if ftype == "fail":
            ch.failed = True
        else:
            ch.fault = ftype

    def fail_gyro(self, idx, ftype="fail"):
        ch = self.gyro[idx]
        ch.clear_fault()
        if ftype == "fail":
            ch.failed = True
        else:
            ch.fault = ftype

    def recover_prs(self, idx):
        self.prs[idx].clear_fault()
        self.prs[idx].restore()

    def recover_gyro(self, idx):
        self.gyro[idx].clear_fault()
        self.gyro[idx].restore()

    def deselect_prs(self, idx, on=True):
        self.prs[idx].deselected = on

    def deselect_gyro(self, idx, on=True):
        self.gyro[idx].deselected = on

    def restore_prs(self, idx):
        self.prs[idx].restore()

    def restore_gyro(self, idx):
        self.gyro[idx].restore()

    # --------------------------------------------------- доступность
    def prs_available(self):
        return sum(1 for ch in self.prs if ch.active)

    def gyro_available(self):
        return sum(1 for ch in self.gyro if ch.active)

    # --------------------------------------------------- чтение
    def read_position(self, eta_true, t=None):
        """Свотированное измерение [x, y] или None, если систем нет."""
        readings = {}
        for i, ch in enumerate(self.prs):
            r = ch.read(eta_true[0:2], t or 0.0, self.rng)
            if r is not None:
                readings[i] = r
        # медиана по активным (кроме проверяемого) — эталон для теста
        act = [i for i, ch in enumerate(self.prs) if ch.active]
        for i, ch in enumerate(self.prs):
            if ch.failed or ch.deselected or i not in readings:
                continue
            others = [readings[j] for j in act if j != i and j in readings]
            # тест по медиане — только при ≥2 других системах (иначе
            # здоровая система рискует быть отбракованной по глючной)
            med = (np.median(np.array(others), axis=0)
                   if len(others) >= 2 else None)
            if not ch.rejected:
                why = ch.test(t or 0.0, med)
                if why:
                    ch.rejected = True
                    ch.reason = why
        # две системы в работе и разошлись — авария расхождения без
        # отбраковки: какую снимать, решает оператор
        act = [i for i, ch in enumerate(self.prs)
               if ch.active and i in readings]
        for ch in self.prs:
            ch.disagree = False
        if len(act) == 2:
            d = float(np.hypot(*(readings[act[0]] - readings[act[1]])))
            if d > DEV_LIM:
                for i in act:
                    self.prs[i].disagree = True
        # веса и вотирование по активным
        act = [i for i, ch in enumerate(self.prs)
               if ch.active and i in readings]
        if not act:
            self._last_est = None
            return None
        for i in act:
            ch = self.prs[i]
            if self._last_est is not None:
                res = float(np.hypot(*(readings[i] - self._last_est)))
                ch.sigma = max(0.05, 0.98 * ch.sigma + 0.02 * res)
        w = np.array([1.0 / self.prs[i].sigma ** 2 for i in act])
        w /= w.sum()
        for k, i in enumerate(act):
            self.prs[i].weight = float(w[k])
        for ch in self.prs:
            if not ch.active:
                ch.weight = 0.0
        est = sum(w[k] * readings[i] for k, i in enumerate(act))
        self._last_est = est
        return est

    def read_heading(self, psi_true, t=None):
        """Принятый курс (среднее активных) или None, если гиро нет."""
        readings = {}
        for i, ch in enumerate(self.gyro):
            r = ch.read(psi_true, t or 0.0, self.rng)
            if r is not None:
                readings[i] = r
        for i, ch in enumerate(self.gyro):
            if ch.failed or ch.deselected or i not in readings:
                continue
            if not ch.rejected:
                why = ch.test()
                if why:
                    ch.rejected = True
                    ch.reason = why
        act = [i for i, ch in enumerate(self.gyro)
               if ch.active and i in readings]
        for ch in self.gyro:
            ch.used = False
        if not act:
            return None
        used = float(np.mean([readings[i] for i in act]))
        # расхождение пары
        if len(act) == 2:
            d = float(np.rad2deg(abs(readings[act[0]]
                                     - readings[act[1]])))
            for i in act:
                self.gyro[i].disagree = d > GYRO_DISAGREE_DEG
        else:
            self.gyro[act[0]].disagree = False
        for i, ch in enumerate(self.gyro):
            if i in readings:
                ch.dev_deg = float(np.rad2deg(readings[i] - used))
        for i in act:
            self.gyro[i].used = True
        return used

    # --------------------------------------------------- телеметрия
    def prs_detail(self, t=None):
        out = []
        for ch in self.prs:
            xy = ch.hist[-1][1] if ch.hist else None
            age = (round(t - ch.hist[-1][0], 1)
                   if t is not None and ch.hist else None)
            out.append({"name": ch.name, "state": ch.state,
                        "reason": ch.reason,
                        "weight_pct": round(100 * ch.weight, 1),
                        "sigma_m": round(ch.sigma, 2),
                        "disagree": ch.disagree,
                        # возраст последнего измерения, с
                        "age_s": age,
                        # последнее измерение (NED), для меток на плане
                        "xy": None if xy is None else [round(float(xy[0]), 2),
                                                       round(float(xy[1]), 2)]})
        return out

    def gyro_detail(self, t=None):
        out = []
        for ch in self.gyro:
            age = (round(t - ch.hist[-1][0], 1)
                   if t is not None and ch.hist else None)
            out.append({"name": ch.name, "state": ch.state,
                        "reason": ch.reason, "used": ch.used,
                        "disagree": ch.disagree,
                        "dev_deg": round(ch.dev_deg, 2),
                        "age_s": age})
        return out
