"""Проверка: ограничение bias наблюдателя и ROT на полном моменте джойстика."""
import math
import sys

sys.path.insert(0, ".")
from console import DPConsole  # noqa: E402

c = DPConsole(wind_speed=8.0, wind_dir=45.0)

# 1) Bias: прогон 120 с, bias не должен превышать лимиты clamp
for _ in range(1200):
    c.step()
b = c.observer.bias
print(f"bias через 120 с: [{b[0]/1e3:.1f}, {b[1]/1e3:.1f}, {b[2]/1e3:.1f}] кН")
lim = (1.5e5, 1.5e5, 4.0e5)
assert all(abs(b[i]) <= lim[i] * 1.01 for i in range(3)), "bias вышел за clamp!"

# 2) ROT на полном моменте: MAN JOY, m=1.0, 60 с на раскрутку, затем измерение
c.set_mode("man_joy")
c.set_joystick(0.0, 0.0, 1.0)
for _ in range(900):  # 90 с — установившийся режим
    c.step()
rot = abs(math.degrees(c.nu[2]) * 60.0)  # °/мин, установившаяся скорость
print(f"ROT на полном моменте: {rot:.0f} °/мин")
assert rot <= 150.0, f"ROT {rot:.0f} > 150 °/мин"
assert rot >= 30.0, f"ROT {rot:.0f} < 30 °/мин — момент слишком вялый"

# 3) Сила вперёд на полном ходу: должна дать разумную скорость
c.set_joystick(1.0, 0.0, 0.0)
for _ in range(600):
    c.step()
spd = math.hypot(c.nu[0], c.nu[1]) * 1.94384
print(f"Скорость на полном fx: {spd:.1f} уз")
assert 1.0 <= spd <= 8.0, f"скорость {spd:.1f} уз вне разумного диапазона"

print("ВСЕ ПРОВЕРКИ ПРОШЛИ")
