// Типы данных ядра СДП: описание судна, движителей, внешней среды, датчиков и команд.
#pragma once
#include <string>
#include <vector>

#include "dp/math.hpp"

namespace dp {

constexpr double RHO_WATER = 1025.0;  // кг/м³
constexpr double RHO_AIR = 1.225;     // кг/м³
constexpr double G = 9.80665;

// ---------------------------------------------------------------- движители
enum class ThrusterType { Fixed, Tunnel, Azimuth };

struct ForbiddenSector {  // запретный сектор угла азимутального движителя, градусы от носа по часовой
    double fromDeg = 0, toDeg = 0;
};

struct AnglePt { double angleDeg = 0, pct = 100; };   // точка таблицы КПД по углу упора (угол упора на судно: 0 — вперёд, 90 — вправо), % от максимума
struct CurvePt { double rpm = 0, thrust = 0, power = 0; };   // точка кривой движителя, все значения в % от максимума

struct ThrusterCfg {
    std::string id;
    ThrusterType type = ThrusterType::Fixed;
    bool enabled = true;
    double x = 0, y = 0, z = 0;       // место установки, м (x к носу, y на правый борт)
    double fixedAngleDeg = 0;         // направление упора фиксированного движителя (0 — вперёд, 90 — вправо)
    double maxForward = 0;            // максимальный упор вперёд (по направлению), Н
    double maxReverse = 0;            // максимальный упор назад, Н
    double timeConstS = 2.0;          // постоянная времени отклика упора, с
    double rateFracPerS = 0.25;       // скорость изменения упора, доля от максимума в секунду
    double azRateDegS = 10.0;         // скорость поворота азимутальной колонки, °/с
    std::vector<ForbiddenSector> forbidden;
    double hardCoreDeg = 360;         // ширина «жёсткого» ядра запретного сектора вокруг его середины, °; остальная часть сектора «нежелательная»: колонка проходит её и может остаться в ней, только если иначе не выполнить запрос (360 — весь сектор жёсткий)
    double parkDeg = 1e9;             // стояночный угол колонки в режиме ожидания, ° (1e9 — не задан, колонка остаётся где была)
    double prefDeg = 1e9;             // предпочтительный угол колонки при удержании, ° (1e9 — не задан); вместе с preloadPct даёт разнонаправленный упор ВРК
    double preloadPct = 0.0;          // предварительный упор вдоль prefDeg, % максимального: пара колонок взаимно гасит поперечную силу и момент
    double fbDiffPct = 25.0;          // порог расхождения упора команды и обратной связи, % максимального (тревога THR_FB_MISMATCH_*)
    double fbDiffAngleDeg = 30.0;     // то же для угла колонки, ° (проверяется при команде упора выше 10 %)
    double fbDiffDelayS = 30.0;       // выдержка расхождения до тревоги, с
    double azDeadDeg = 0.0;           // мягкая зона нечувствительности команды угла, ° (малые довороты не отрабатываются)
    double pctDeadPct = 0.0;          // то же для команды упора, % максимального (для НПУ и колонок)
    double weight = 1.0;              // вес при распределении (больше — движитель загружается меньше)
    double powerMaxKW = 0.0;          // мощность при максимальном упоре, кВт (для ограничения по электростанции)
    int bus = -1;                     // шина питания: 0 — A, 1 — B, -1 — не задана (нагрузка делится поровну)
    std::string failGroups;           // группы наихудшего отказа через «;» (например «ГРЩ-1; МО-1»): движители одной группы отказывают вместе (анализ последствий)
    double limitPct = 100.0;          // предел нагрузки движителя, % максимального упора (100 — без ограничения)
    double idlePct = 0.0;             // холостая мощность при нулевом упоре, % powerMaxKW (ПУ с ВРШ на постоянных оборотах; 0 — нет)
    double fadeKn = 0.0;              // скорость судна, выше которой упор падает, уз (0 — спад не учитывается; для ПУ)
    double fadeFloorPct = 50.0;       // остаточный упор при скорости 2·fadeKn и выше, % (между fadeKn и 2·fadeKn — линейно)
    double speedFactor = 1.0;         // текущий множитель максимального упора по скорости судна (расчётное, не настройка)
    std::vector<AnglePt> effAngle;    // необязательная таблица КПД по углу упора колонки, % (пусто — 100 % по всем углам; промежуточные углы — линейно по кругу)
    bool effIsLimit = false;          // false — потеря упора (физически упор меньше, распределитель компенсирует командой); true — предел команды (упор не выше значения таблицы)
    std::vector<CurvePt> curve;       // необязательная кривая «обороты — упор — мощность», % (пусто — упор линеен по команде, мощность ~ упор^1,5)
};

// ---------------------------------------------------------------- судно
struct WindTable {  // коэффициенты по углу набегающего ветра 0..180° (шаг задан в stepDeg), необязательно
    double stepDeg = 10;
    std::vector<double> cx, cy, cn;
};

struct VesselCfg {
    std::string name = "Судно";
    double length = 60, beam = 14, draft = 4;       // м
    double mass = 2.2e6;                            // кг
    double Ixx = 0, Iyy = 0, Izz = 0;               // моменты инерции, кг·м² (0 — оценить по размерам)
    // присоединённые массы (абсолютные значения; 0 — оценить по доле массы)
    double addSurge = 0, addSway = 0, addHeave = 0, addRoll = 0, addPitch = 0, addYaw = 0;
    // линейное демпфирование: X = -Xu·u и т.д. (0 — оценить)
    double Xu = 0, Yv = 0, Zw = 0, Kp = 0, Mq = 0, Nr = 0;
    // квадратичное демпфирование
    double Xuu = 0, Yvv = 0, Nrr = 0;
    double gmT = 1.5, gmL = 60;                     // метацентрические высоты, м
    double waterplaneCoef = 0.8;                    // коэффициент полноты ватерлинии
    // ветер
    double frontalArea = 120, lateralArea = 380;    // площади надводной части, м²
    double cx = 0.7, cy = 0.9, cn = 0.12;           // коэффициенты (если нет таблицы)
    WindTable windTable;                            // необязательная таблица (приоритетнее)
    // волны
    double waveReflection = 0.3;                    // коэффициент отражения (для дрейфовой силы)
    double raoHeave = 0.9, raoRoll = 0.7, raoPitch = 0.5;
    // Гидроаэродинамика по Руководству РС (DP Capability Plots, ДУ1) и DNV-ST-0111:
    double lpp = 0;                                 // длина между перпендикулярами, м (0 — 0,92·length)
    double los = 0;                                 // длина подводной части по ватерлинии, м (0 — lpp)
    double xLos = 0;                                // смещение середины los от миделя, м
    double waveBeta = 0.8;                          // угол носовой оконечности β, рад (полнота носа; оценка)
    double waveCwla = 1.0;                          // коэффициент полноты кормовой части ватерлинии, 0,85…1,15
    int waveModel = 1;                              // 1 — формулы РС/DNV (по Hs и Tz), 0 — упрощённая (по waveReflection)
    double xWind = 0;                               // продольная координата центра парусности, м (+ к носу)
    bool windMomentRS = false;                      // момент ветра по РС: Cmz = Cy·(x/Lpp + 0,3·(1 − 2α/π)); иначе cn·sin 2α
    double currentLateralArea = 0;                  // площадь боковой проекции подводной части, м² (0 — 0,85·L·T)
    double xCurrent = 0;                            // продольная координата центра этой площади, м
    bool currentRS = false;                         // течение по РС: Cx = −0,07·cos α, Cy = −0,6·sin α и момент; иначе прежняя оценка
};

// ---------------------------------------------------------------- внешняя среда (для имитатора)
struct EnvCfg {
    double windSpeed = 0, windFromDeg = 0;          // истинный ветер, м/с, откуда дует
    double currentSpeed = 0, currentToDeg = 0;      // течение, м/с, куда направлено
    double hs = 0, tp = 8, waveFromDeg = 0;         // высота значительная, период, откуда идут волны
    double turbulence = 0;                          // 0 — идеально ровная среда; 1 — порывы ветра, переменное течение и медленный волновой дрейф; до 2
};

// ---------------------------------------------------------------- датчики
enum class PosType { Gnss, LaserRadar, TautWire, Hydroacoustic };

struct PosMeas {
    std::string id;
    PosType type = PosType::Gnss;
    bool fresh = false;       // true — получено новое измерение на этом шаге
    double n = 0, e = 0;      // положение антенны/цели в локальных координатах NED, м
    double sigma = 1.0;       // СКО, м
    V3 offset;                // смещение приёмника от точки управления, м (связанные оси)
};

struct HdgMeas {
    std::string id;
    bool fresh = false;
    double psi = 0;           // курс, рад
};

struct WindMeas {
    std::string id;
    bool fresh = false;
    double speed = 0;         // кажущийся ветер, м/с
    double fromRel = 0;       // откуда дует относительно носа, рад (по часовой)
};

// Датчик движения судна (МИО, MRU). Внутреннее соглашение о знаках:
//   крен — положительный, когда правый борт опускается; дифферент — положительный, когда нос поднимается;
//   вертикальная качка (heave) — положительная ВНИЗ, м (как в K-Pos). Знаки реального прибора приводятся к этому в ChannelCfg.
//   Внутри оценщика волн используется «вверх» (знак меняется на входе в waves_.push).
struct MruMeas {
    std::string id;
    bool fresh = false;
    double roll = 0, pitch = 0, heave = 0;   // рад, рад, м
};

struct PowerMeas {
    bool fresh = false;
    double availableKW = 1e9;  // доступная мощность электростанции
};

struct ThrusterFb {            // обратная связь от движителя
    std::string id;
    bool valid = false;
    double pct = 0;            // фактический упор, % (−100..100)
    double angleDeg = 0;       // фактический угол азимута
};

struct SensorFrame {
    double time = 0;
    std::vector<PosMeas> pos;
    std::vector<HdgMeas> hdg;
    std::vector<WindMeas> wind;
    std::vector<MruMeas> mru;               // обычно три прибора: MRU1..MRU3
    PowerMeas power;
    std::vector<ThrusterFb> fb;
};

// ---------------------------------------------------------------- команды на движители
struct ThrusterCmd {
    std::string id;
    double pct = 0;            // −100..100 % от максимального упора в соответствующую сторону
    double angleDeg = 0;       // заданный угол (для азимутальных)
};

struct Wrench {                // обобщённая сила в связанных осях: сила вперёд, вправо, момент рысканья (по часовой)
    double X = 0, Y = 0, N = 0;
};

}  // namespace dp
