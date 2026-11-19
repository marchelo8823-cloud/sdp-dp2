// Рабочая программа ядра СДП (консольная). Запускается без данных и ждёт их:
//   ship load <книга судна.xlsx>      — данные судна;
//   sim start <книга имитатора.xlsx>  — имитатор вместо датчиков (необязательно).
// Параметры: --check <книга.xlsx> (только проверка книги судна), --log <файл.csv>, --hz <частота цикла>,
//            --duration <с> (остановка по времени ядра), --quiet (без строки состояния), --script <файл> (команды из файла).
#include <atomic>
#include <chrono>
#include <clocale>
#include <csignal>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

#include "dp/core.hpp"

using namespace dp;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop = true; }

int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "C");
    CoreSettings st; double duration = -1, statusPeriod = 10; bool quiet = false; std::string check, script;
    const char* usage = "Использование: sdp_run [--check <книга судна.xlsx>] [--log <файл.csv>] [--hz <Гц>] [--duration <с>] [--quiet] [--script <файл команд>]\n";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--check" && i + 1 < argc) check = argv[++i];
        else if (a == "--log" && i + 1 < argc) st.logFile = argv[++i];
        else if (a == "--hz" && i + 1 < argc) st.cycleHz = std::atof(argv[++i]);
        else if (a == "--duration" && i + 1 < argc) duration = std::atof(argv[++i]);
        else if (a == "--status" && i + 1 < argc) statusPeriod = std::atof(argv[++i]);
        else if (a == "--script" && i + 1 < argc) script = argv[++i];
        else if (a == "--quiet") quiet = true;
        else if (a == "--help" || a == "-h") { std::printf("%s%s", usage, Core::helpText()); return 0; }
        else { std::printf("Неизвестный параметр: %s\n%s", a.c_str(), usage); return 2; }
    }
    if (!check.empty()) {   // проверка книги без запуска ядра
        Core c(st);
        const BookReport r = c.checkShip(check);
        for (auto& e : r.errors) std::printf("ОШИБКА: %s\n", e.c_str());
        for (auto& w : r.warnings) std::printf("Предупреждение: %s\n", w.c_str());
        for (auto& d : r.defaults) std::printf("По умолчанию: %s\n", d.c_str());
        for (auto& e : r.estimates) std::printf("Оценка: %s\n", e.c_str());
        std::printf(r.ok() ? "Книга в порядке.\n" : "В книге ошибок: %zu.\n", r.errors.size());
        return r.ok() ? 0 : 2;
    }
    if (st.cycleHz < 1 || st.cycleHz > 100) { std::printf("ОШИБКА: --hz от 1 до 100\n"); return 2; }

    std::signal(SIGINT, onSignal);
    Core core(st, [](const std::string& s) { std::printf("%s\n", s.c_str()); std::fflush(stdout); });

    std::mutex qm; std::deque<std::string> queue;
    if (!script.empty()) {   // команды из файла; строка «wait <с>» — пауза по времени ядра
        std::ifstream f(script); if (!f) { std::printf("ОШИБКА: не открыт файл команд %s\n", script.c_str()); return 2; }
        std::string l; while (std::getline(f, l)) if (!l.empty() && l[0] != '#') queue.push_back(l);
    } else {
        std::thread([&] {
            std::string line;
            while (std::getline(std::cin, line)) { std::lock_guard<std::mutex> lk(qm); queue.push_back(line); }
        }).detach();
    }

    std::printf("Ядро СДП запущено. Состояние: %s — загрузите судно: ship load <книга.xlsx>. help — список команд, Ctrl+C — выход.\n", coreStateName(core.state()));
    std::fflush(stdout);
    double nextStatus = statusPeriod, waitUntil = -1;
    auto next = std::chrono::steady_clock::now();
    while (!g_stop && !core.quitRequested()) {
        next += std::chrono::microseconds(static_cast<long long>(1e6 * core.dt() / core.simSpeed()));
        if (core.clock() >= waitUntil) {
            std::deque<std::string> q; { std::lock_guard<std::mutex> lk(qm); q.swap(queue); }
            while (!q.empty()) {
                const std::string l = q.front(); q.pop_front();
                if (l.rfind("wait ", 0) == 0) { waitUntil = core.clock() + std::atof(l.c_str() + 5); std::lock_guard<std::mutex> lk(qm); for (auto it = q.rbegin(); it != q.rend(); ++it) queue.push_front(*it); break; }
                std::printf("> %s\n", l.c_str()); core.command(l);
            }
        }
        core.cycle();
        if (core.takeStatusRequest()) nextStatus = core.clock();
        if (!quiet && core.clock() >= nextStatus) {
            nextStatus = core.clock() + statusPeriod;
            const EngineOutput& o = core.out();
            if (!core.hasShip()) std::printf("[t=%6.0f с] %s\n", core.clock(), coreStateName(core.state()));
            else {
                std::string al; for (auto& a : o.alarms) al += (al.empty() ? "" : ", ") + a.code;
                std::string ln; for (auto& s : core.linkStatus()) ln += " " + s.id + (s.connected ? ":связь" : ":НЕТ СВЯЗИ") + (s.stale ? "(команды устарели)" : "");
                std::printf("[t=%6.0f с] %s%s | %s | N=%.1f E=%.1f курс=%.1f° | ошибка %.2f м, %.2f° | ветер %.1f м/с | загрузка %.0f %% | тревоги: %s |%s\n",
                            core.clock(), coreStateName(core.state()), core.simRunning() ? " (ИМИТАЦИЯ)" : "", modeName(core.mode()), o.est.n, o.est.e, rad2deg(o.est.psi), o.ctl.errDist, o.ctl.errHdgDeg,
                            o.est.windSpeed, 100 * o.alloc.utilization, al.empty() ? "нет" : al.c_str(), core.simRunning() ? " выдача: имитатор" : (ln.empty() ? " выдача не настроена" : ln.c_str()));
            }
            std::fflush(stdout);
        }
        if (duration > 0 && core.clock() >= duration) break;
        if (!script.empty() && queue.empty() && core.clock() >= waitUntil && duration <= 0) break;
        std::this_thread::sleep_until(next);
        if (std::chrono::steady_clock::now() > next + std::chrono::seconds(1)) next = std::chrono::steady_clock::now();
    }
    core.stop();
    std::printf("Остановлено на %.0f с.\n", core.clock());
    const RunStats s = core.stats();
    if (s.seconds > 0) std::printf("ИТОГ после переходного процесса (%.0f с): ошибка положения среднеквадратичная %.2f м, наибольшая %.2f м; курса %.2f°, наибольшая %.2f°.\n", s.seconds, s.rmsPos, s.maxPos, s.rmsHdg, s.maxHdg);
    std::fflush(stdout);
    std::_Exit(0);
}
