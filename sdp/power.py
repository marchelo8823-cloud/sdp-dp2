"""Электростанция судна (PMS): генераторы, шины, ограничение тяги.

По конфигурации Praxis (sdp2_praxis.json): шесть ДГ по 1000 кВт на
двух шинах (DG1–3 — шина A, DG4–6 — шина B), секционный автомат
(перемычка) замкнут, собственные нужды 300 кВт на шину, резерв 15 %.

Связь с движителями: потребляемая мощность ДРУ P = P_max·(|u|/u_max)²
(квадратичная зависимость — типовая оценка для ВФШ/ПУ; P_max — шкала
датчика мощности из конфига). Если запрос шины превышает доступную
мощность, PMS снижает допустимую тягу каждого движителя пропорционально
(аналог тега Tn_RPM_PMS_REDUCE в Praxis): u_avail = u_max·√s.

Отказ шины: потребители мёртвой шины обестечены; при замкнутой
перемычке их питает соседняя шина (если хватает мощности), при
разомкнутой — движители шины встают. Классический сценарий DP-2.
"""

GEN_START_S = 30.0       # запуск ДГ до приёма нагрузки
RESERVE_MIN_PCT = 15.0   # минимальный spinning reserve (предупреждение)


class Generator:
    def __init__(self, name, rated_kw, bus):
        self.name = name
        self.rated_kw = rated_kw
        self.bus = bus
        self.online = True        # включён оператором
        self.failed = False       # аварийный отказ
        self.start_left = 0.0     # остаток времени запуска, с

    @property
    def avail_kw(self):
        if self.failed or not self.online or self.start_left > 0.0:
            return 0.0
        return self.rated_kw

    def step(self, dt):
        if self.start_left > 0.0:
            self.start_left = max(0.0, self.start_left - dt)

    def start(self):
        if not self.failed and not self.online:
            self.online = True
            self.start_left = GEN_START_S

    def stop(self):
        self.online = False
        self.start_left = 0.0

    def snapshot(self):
        if self.failed:
            st = "failed"
        elif not self.online:
            st = "stopped"
        elif self.start_left > 0.0:
            st = "starting"
        else:
            st = "online"
        return {"name": self.name, "bus": self.bus, "state": st,
                "rated_kw": self.rated_kw,
                "start_left": round(self.start_left, 0)}


class PowerPlant:
    """Две шины A/B + секционный автомат; питание движителей."""

    def __init__(self, profile, thrusters):
        self.thrusters = thrusters
        if profile == "sdp2":
            # sdp2_praxis.json: DG1–6 по 1000 кВт; THR1..8 попеременно
            # на шинах A/B; P_max — шкалы датчиков мощности
            self.gens = [Generator(f"DG{i + 1}", 1000.0,
                                   "A" if i < 3 else "B")
                         for i in range(6)]
            self.service_kw = {"A": 300.0, "B": 300.0}
            self.p_max_kw = [800.0, 800.0, 460.0, 400.0,
                             460.0, 460.0, 800.0, 800.0]
            self.bus_of = ["A", "B", "A", "B", "A", "B", "A", "B"]
        else:
            # учебный 4dru: 2×2 ДГ по 2000 кВт; P_max из эвристики
            # 150 Н/кВт (та же, что в заглушках sdp2_praxis.json)
            self.gens = [Generator("DG1", 2000.0, "A"),
                         Generator("DG2", 2000.0, "A"),
                         Generator("DG3", 2000.0, "B"),
                         Generator("DG4", 2000.0, "B")]
            self.service_kw = {"A": 250.0, "B": 250.0}
            self.p_max_kw = [th.t_max / 150.0 for th in thrusters]
            self.bus_of = ["A", "B", "A", "B"]
        self.tie_closed = True
        self.bus_dead = {"A": False, "B": False}
        self.min_reserve_pct = RESERVE_MIN_PCT
        self.limit_mask = [False] * len(thrusters)   # PMS reduce активен
        self.load_kw = 0.0
        self.avail_kw = 0.0

    # ---------------------------------------------------------- расчёт
    def _networks(self):
        """Действующие сети: {сеть: (шины-источники, шины-потребители)}.

        Перемычка замкнута → единая сеть (мёртвая шина без генерации,
        но её потребители питаются через перемычку). Разомкнута → две
        независимые сети; мёртвая шина обесточена полностью.
        """
        if self.tie_closed:
            return {"AB": (["A", "B"], ["A", "B"])}
        return {"A": (["A"], ["A"]), "B": (["B"], ["B"])}

    def step(self, dt):
        """Пересчёт допустимой тяги th.u_avail по балансу мощности."""
        for g in self.gens:
            g.step(dt)
        nets = self._networks()
        self.load_kw = 0.0
        self.avail_kw = 0.0
        self.limit_mask = [False] * len(self.thrusters)
        # u_avail по умолчанию — полный упор; дальше режем
        for net, (src_buses, load_buses) in nets.items():
            avail = sum(g.avail_kw for g in self.gens
                        if g.bus in src_buses
                        and not self.bus_dead[g.bus])
            service = sum(self.service_kw[b] for b in load_buses)
            budget = max(0.0, avail - service)
            idx = [i for i, b in enumerate(self.bus_of)
                   if b in load_buses]
            demand = 0.0
            dem = {}
            for i in idx:
                th = self.thrusters[i]
                if th.failed:
                    dem[i] = 0.0
                    continue
                p = self.p_max_kw[i] * (abs(th.u) / th.t_max) ** 2
                dem[i] = p
                demand += p
            self.load_kw += demand + service
            self.avail_kw += avail
            for i in idx:
                th = self.thrusters[i]
                if avail <= 0.0:
                    th.u_avail = 0.0
                    self.limit_mask[i] = not th.failed
                elif demand > budget and demand > 1e-6:
                    s = max(0.0, budget / demand)
                    th.u_avail = th.t_max * s ** 0.5
                    self.limit_mask[i] = True
                else:
                    th.u_avail = th.t_max

    # ---------------------------------------------------------- события
    def fail_gen(self, i):
        self.gens[i].failed = True
        self.gens[i].online = False

    def recover_gen(self, i):
        self.gens[i].failed = False
        self.gens[i].online = True
        self.gens[i].start_left = GEN_START_S   # после аварии — пуск

    def fail_bus(self, name):
        self.bus_dead[name] = True

    def recover_bus(self, name):
        self.bus_dead[name] = False

    def failure_groups(self):
        """Группы движителей, выбывающие одним отказом (для CA).

        При разомкнутой перемычке отказ шины снимает её движители —
        два дополнительных сценария единичного отказа.
        """
        if self.tie_closed:
            return []
        return [[i for i, b in enumerate(self.bus_of) if b == name]
                for name in ("A", "B")]

    # ---------------------------------------------------------- статус
    @property
    def reserve_pct(self):
        """Spinning reserve: избыток доступной мощности над нагрузкой."""
        if self.avail_kw <= 0.0:
            return 0.0
        return max(0.0, 100.0 * (self.avail_kw - self.load_kw)
                   / self.avail_kw)

    def snapshot(self):
        buses = []
        for name in ("A", "B"):
            idx = [i for i, b in enumerate(self.bus_of) if b == name]
            load = sum(self.p_max_kw[i]
                       * (abs(self.thrusters[i].u)
                          / self.thrusters[i].t_max) ** 2
                       for i in idx if not self.thrusters[i].failed)
            avail = (0.0 if self.bus_dead[name]
                     else sum(g.avail_kw for g in self.gens
                              if g.bus == name))
            buses.append({"name": name,
                          "load_kw": round(load + self.service_kw[name], 0),
                          "avail_kw": round(avail, 0),
                          "dead": self.bus_dead[name]})
        return {"gens": [g.snapshot() for g in self.gens],
                "buses": buses,
                "tie_closed": self.tie_closed,
                "load_kw": round(self.load_kw, 0),
                "avail_kw": round(self.avail_kw, 0),
                "reserve_pct": round(self.reserve_pct, 0),
                "limit_mask": list(self.limit_mask)}
