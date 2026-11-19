"""Профили судов: наборы движителей.

«4dru» — учебный DP-2 (2 носовых туннельных + 2 кормовых азимутальных),
на нём настроены trials и регуляторы.

«sdp2» — судно СДП-2 по конфигурации Praxis Mega-Guard
(reference/incoming_1118/sdp2_praxis.json):
  THR1, THR2 — главные ВРШ (неповоротные, реверс 25 %, темп 15 %/с);
  THR3, THR4 — носовые ПУ; THR5, THR6 — кормовые ПУ;
  THR7, THR8 — поворотные (азимутальные).
⚠ Корпус в обоих профилях общий (Vessel3DOF): размеры/масса СДП-2
в выгрузке Praxis отсутствуют (в JSON помечены ЗАГЛУШКА), поэтому
профиль меняет только электродвижительную установку.
"""
from .allocation import AzimuthThruster
from .simcore import default_thrusters

PROFILES = ("4dru", "sdp2")


def sdp2_thrusters():
    """8 движителей СДП-2; координаты и упоры — ЗАГЛУШКИ из sdp2_praxis.json
    (упор ~150 Н/кВт от шкалы датчика мощности)."""
    def T(**kw):
        return AzimuthThruster(**kw)

    return [
        # THR1, THR2 — главные ВРШ: ось вдоль судна, реверс 25 %,
        # темп шага 15 %/с
        T(lx=-34.0, ly=-4.0, t_max=1.2e5, t_min=-0.3e5, az0_deg=0.0,
          tunnel=True, thrust_rate=0.15 * 1.2e5, name="THR1"),
        T(lx=-34.0, ly=4.0, t_max=1.2e5, t_min=-0.3e5, az0_deg=0.0,
          tunnel=True, thrust_rate=0.15 * 1.2e5, name="THR2"),
        # THR3, THR4 — носовые ПУ
        T(lx=32.0, ly=0.0, t_max=0.69e5, az0_deg=90.0, tunnel=True,
          thrust_rate=0.15 * 0.69e5, name="THR3"),
        T(lx=28.0, ly=0.0, t_max=0.60e5, az0_deg=90.0, tunnel=True,
          thrust_rate=0.15 * 0.60e5, name="THR4"),
        # THR5, THR6 — кормовые ПУ
        T(lx=-26.0, ly=0.0, t_max=0.69e5, az0_deg=90.0, tunnel=True,
          thrust_rate=0.15 * 0.69e5, name="THR5"),
        T(lx=-30.0, ly=0.0, t_max=0.69e5, az0_deg=90.0, tunnel=True,
          name="THR6"),
        # THR7, THR8 — поворотные (азимут 12°/с, без запрещённых зон)
        T(lx=20.0, ly=-5.0, t_max=1.2e5, az0_deg=60.0, az_rate_deg_s=12.0,
          servo_err_deg=-1.2, name="THR7"),
        T(lx=20.0, ly=5.0, t_max=1.2e5, az0_deg=-60.0, az_rate_deg_s=12.0,
          servo_err_deg=1.6, name="THR8"),
    ]


def build_thrusters(profile="4dru"):
    if profile == "sdp2":
        return sdp2_thrusters()
    if profile == "4dru":
        return default_thrusters()
    raise ValueError(f"неизвестный профиль: {profile!r} (есть {PROFILES})")
