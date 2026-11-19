// sdp_ship — редактор данных судна для ядра СДП: таблицы книги Excel и чертёж из простых фигур
// с подложкой-фотографией, подсказкой контура и распознаванием через Claude.
// Запуск: sdp_ship [книга.xlsx]
// Для проверок без экрана (QT_QPA_PLATFORM=offscreen): --tab N, --underlay N=файл, --calib N=px1,py1,px2,py2,a1,b1,a2,b2,
//   --contour N=x0,y0,x1,y1, --apply N, --check, --save файл, --screenshot файл, --size ШxВ
#include <QApplication>
#include <QScreen>
#include <QEventLoop>
#include <QTimer>
#include <cstdio>

#include "mainwindow.h"
#include "viewedit.h"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("sdp_ship"); QApplication::setOrganizationName("SDP");
    MainWindow w;
    for (const QString& x : app.arguments()) if (x == "--screenshot") w.quietDialogs = true;
    QStringList a = app.arguments(); a.removeFirst();
    QString shot, save; QSize size(1400, 860); int tab = -1;
    struct Step { QString kind; int view; QString arg; };
    QVector<Step> steps;
    for (int i = 0; i < a.size(); ++i) {
        const QString s = a[i];
        auto next = [&] { return i + 1 < a.size() ? a[++i] : QString(); };
        auto viewArg = [&](const QString& kind) { const QString x = next(); const int eq = x.indexOf('='); steps.push_back({kind, x.left(eq).toInt(), eq >= 0 ? x.mid(eq + 1) : QString()}); };
        if (s == "--screenshot") shot = next();
        else if (s == "--save") save = next();
        else if (s == "--size") { const QStringList p = next().split('x'); if (p.size() == 2) size = QSize(p[0].toInt(), p[1].toInt()); }
        else if (s == "--tab") tab = next().toInt();
        else if (s == "--underlay") viewArg("underlay");
        else if (s == "--calib") viewArg("calib");
        else if (s == "--contour") viewArg("contour");
        else if (s == "--apply") steps.push_back({"apply", next().toInt(), {}});
        else if (s == "--check") steps.push_back({"check", 0, {}});
        else if (s == "--recognize") steps.push_back({"recognize", next().toInt(), {}});
        else if (s == "--wait") steps.push_back({"wait", 0, next()});
        else if (s == "--hull") steps.push_back({"hull", 0, next()});        // --hull "Подпись=значение" — ввод в «Корпус»
        else if (!s.startsWith("--")) w.openBook(s);
    }
    w.resize(size); w.show();
    for (const auto& st : steps) {
        const QStringList n = st.arg.split(',');
        if (st.kind == "underlay") w.loadUnderlay(st.view, st.arg);
        else if (st.kind == "calib" && n.size() == 8) w.calibrate(st.view, {n[0].toDouble(), n[1].toDouble()}, {n[2].toDouble(), n[3].toDouble()}, {n[4].toDouble(), n[5].toDouble()}, {n[6].toDouble(), n[7].toDouble()});
        else if (st.kind == "contour" && n.size() == 4) w.runContour(st.view, QRectF(QPointF(n[0].toDouble(), n[1].toDouble()), QPointF(n[2].toDouble(), n[3].toDouble())));
        else if (st.kind == "apply") w.applyCalc(st.view);
        else if (st.kind == "check") w.checkBook();
        else if (st.kind == "recognize") w.recognizeView(st.view);
        else if (st.kind == "hull") { const int eq = st.arg.indexOf('='); w.setHullValue(st.arg.left(eq), st.arg.mid(eq + 1)); }
        else if (st.kind == "wait") { QEventLoop l; QTimer::singleShot(int(st.arg.toDouble() * 1000), &l, &QEventLoop::quit); l.exec(); }
    }
    if (tab >= 0) w.setTab(tab);
    if (!save.isEmpty()) w.saveBook(save);
    if (!shot.isEmpty()) {
        QTimer::singleShot(600, &w, [&] {
            for (int v = 0; v < 3; ++v) w.viewEdit(v)->fit();
            QTimer::singleShot(300, &w, [&] { w.grab().save(shot); std::printf("снимок: %s\n", shot.toLocal8Bit().constData()); std::fflush(stdout); QCoreApplication::exit(0); });
        });
    }
    return app.exec();
}
