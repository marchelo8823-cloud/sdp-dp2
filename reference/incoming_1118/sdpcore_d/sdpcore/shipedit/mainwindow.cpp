#include "mainwindow.h"

#include <QApplication>
#include <QBuffer>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include "keystore.h"
#include <QSlider>
#include <QStatusBar>
#include <QStyledItemDelegate>
#include <QTabWidget>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <cmath>

#include "dp/contour.hpp"
#include "dp/shipbook.hpp"
#include "dp/vision.hpp"
#include "viewedit.h"

using namespace dp::design;

static QString Q(const std::string& s) { return QString::fromStdString(s); }
static std::string S(const QString& s) { return s.trimmed().toStdString(); }
static bool numOf(const QString& s, double& v) { QString t = s.trimmed(); t.replace(',', '.'); t.remove(' '); bool ok = false; v = t.toDouble(&ok); return ok && !t.isEmpty(); }
static QString fmt(double v, int d) { QString s = QString::number(v, 'f', d); if (s.contains('.')) { while (s.endsWith('0')) s.chop(1); if (s.endsWith('.')) s.chop(1); } return s; }

// Выпадающий список в ячейке таблицы
class ListDelegate : public QStyledItemDelegate {
public:
    ListDelegate(QStringList items, QObject* p) : QStyledItemDelegate(p), items_(std::move(items)) {}
    QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem&, const QModelIndex&) const override { auto* c = new QComboBox(parent); c->addItem(""); c->addItems(items_); return c; }
    void setEditorData(QWidget* e, const QModelIndex& i) const override { auto* c = static_cast<QComboBox*>(e); c->setCurrentText(i.data().toString()); }
    void setModelData(QWidget* e, QAbstractItemModel* m, const QModelIndex& i) const override { m->setData(i, static_cast<QComboBox*>(e)->currentText()); }
private:
    QStringList items_;
};

static const QColor kReq(255, 242, 204), kOpt(226, 239, 218);

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("Данные судна — СДП");
    resize(1400, 860);
    net_ = new QNetworkAccessManager(this);

    auto* tb = addToolBar("Файл"); tb->setMovable(false); tb->setToolButtonStyle(Qt::ToolButtonTextOnly);
    tb->addAction("Новое судно", this, [this] { if (confirmDiscard()) newShip(); });
    tb->addAction("Открыть книгу…", this, [this] {
        if (!confirmDiscard()) return;
        const QString p = QFileDialog::getOpenFileName(this, "Открыть книгу судна", QFileInfo(path_).absolutePath(), "Книга Excel (*.xlsx)");
        if (!p.isEmpty()) openBook(p);
    });
    tb->addAction("Сохранить", this, [this] {
        QString p = path_;
        if (p.isEmpty()) p = QFileDialog::getSaveFileName(this, "Сохранить книгу судна", "Данные_судна.xlsx", "Книга Excel (*.xlsx)");
        if (!p.isEmpty()) saveBook(p);
    });
    tb->addAction("Сохранить как…", this, [this] {
        const QString p = QFileDialog::getSaveFileName(this, "Сохранить книгу судна", path_.isEmpty() ? "Данные_судна.xlsx" : path_, "Книга Excel (*.xlsx)");
        if (!p.isEmpty()) saveBook(p);
    });
    tb->addSeparator();
    tb->addAction("Проверить ядром", this, [this] { checkBook(); });
    tb->addSeparator();
    tb->addAction("Распознавание Claude…", this, [this] { claudeSettings(); });

    tabs_ = new QTabWidget(this); setCentralWidget(tabs_);
    hull_ = kvTable(hullSpec(), true);
    { auto* w = new QWidget; auto* l = new QVBoxLayout(w);
      l->addWidget(new QLabel("Жёлтые — обязательные, зелёные — необязательные. Значения из чертежа переносятся кнопкой «Перенести в корпус» на вкладках видов."));
      l->addWidget(hull_); tabs_->addTab(w, "Корпус"); }
    for (int v = 0; v < 3; ++v) tabs_->addTab(buildView(v), QString("Вид ") + viewName(v));
    auto rowsTab = [&](QTableWidget*& t, const std::vector<ColSpec>& spec, const QString& title, const QString& hint) {
        t = rowTable(spec);
        auto* w = new QWidget; auto* l = new QVBoxLayout(w); auto* h = new QHBoxLayout;
        auto* add = new QPushButton("Добавить строку"); auto* del = new QPushButton("Удалить строку");
        connect(add, &QPushButton::clicked, this, [this, t] { t->insertRow(t->rowCount()); for (int c = 0; c < t->columnCount(); ++c) { auto* it = new QTableWidgetItem; it->setBackground(t->horizontalHeaderItem(c)->data(Qt::UserRole).toBool() ? kReq : kOpt); t->setItem(t->rowCount() - 1, c, it); } setDirty(); });
        connect(del, &QPushButton::clicked, this, [this, t] { const int r = t->currentRow(); if (r >= 0) { t->removeRow(r); setDirty(); refreshMarkers(); } });
        h->addWidget(add); h->addWidget(del); h->addStretch(); h->addWidget(new QLabel(hint));
        l->addLayout(h); l->addWidget(t); tabs_->addTab(w, title);
    };
    rowsTab(thr_, thrSpec(), "Движители", "Место можно задать перетаскиванием метки на виде сверху.");
    thr_->setItemDelegateForColumn(1, new ListDelegate({"туннельный", "поворотный", "неповоротный"}, this));
    thr_->setItemDelegateForColumn(2, new ListDelegate({"да", "нет"}, this));
    rowsTab(sen_, senSpec(), "Датчики", "Место можно задать перетаскиванием метки на виде сверху; Z — в таблице.");
    sen_->setItemDelegateForColumn(1, new ListDelegate({"спутниковый", "гирокомпас", "ветер", "качка", "лазер", "трос"}, this));
    sen_->setItemDelegateForColumn(6, new ListDelegate({"udp", "tcp", "serial"}, this));
    { outKv_ = kvTable(outKvSpec(), false); outKv_->setMaximumHeight(300);
      outRows_ = rowTable(outSpec());
      outRows_->setItemDelegateForColumn(1, new ListDelegate({"упор", "угол"}, this));
      outRows_->setItemDelegateForColumn(4, new ListDelegate({"да", "нет"}, this));
      auto* w = new QWidget; auto* l = new QVBoxLayout(w);
      l->addWidget(new QLabel("Выдача команд движителям по Modbus. Если не заполнено, команды никуда не передаются."));
      l->addWidget(outKv_);
      auto* h = new QHBoxLayout; auto* add = new QPushButton("Добавить регистр"); auto* del = new QPushButton("Удалить регистр");
      connect(add, &QPushButton::clicked, this, [this] { outRows_->insertRow(outRows_->rowCount()); setDirty(); });
      connect(del, &QPushButton::clicked, this, [this] { if (outRows_->currentRow() >= 0) { outRows_->removeRow(outRows_->currentRow()); setDirty(); } });
      h->addWidget(add); h->addWidget(del); h->addStretch(); l->addLayout(h); l->addWidget(outRows_);
      tabs_->addTab(w, "Выдача"); }
    report_ = new QPlainTextEdit; report_->setReadOnly(true);
    { auto* w = new QWidget; auto* l = new QVBoxLayout(w);
      auto* b = new QPushButton("Проверить ядром"); connect(b, &QPushButton::clicked, this, [this] { checkBook(); });
      auto* h = new QHBoxLayout; h->addWidget(b); h->addWidget(new QLabel("Та же проверка, что при ship load в ядре: заполненность, согласованность, оценки.")); h->addStretch();
      l->addLayout(h); l->addWidget(report_); tabs_->addTab(w, "Проверка"); }

    coord_ = new QLabel; statusBar()->addPermanentWidget(coord_);
    for (QTableWidget* t : {hull_, thr_, sen_, outKv_, outRows_})
        connect(t, &QTableWidget::itemChanged, this, [this, t] { if (loading_) return; setDirty(); if (t == thr_ || t == sen_) refreshMarkers(); if (t == hull_ || t == thr_) { refreshGuides(); refreshScheme(); } });
    newShip();
}

// ---------------------------------------------------------------- таблицы
QTableWidget* MainWindow::kvTable(const std::vector<KvSpec>& spec, bool reqCol) {
    auto* t = new QTableWidget(static_cast<int>(spec.size()), reqCol ? 4 : 3);
    t->setHorizontalHeaderLabels(reqCol ? QStringList{"Параметр", "Значение", "Единица", "Пояснение"} : QStringList{"Параметр", "Значение", "Пояснение"});
    t->verticalHeader()->hide(); t->horizontalHeader()->setStretchLastSection(true);
    t->setColumnWidth(0, 300); t->setColumnWidth(1, 220); if (reqCol) t->setColumnWidth(2, 70);
    for (int r = 0; r < static_cast<int>(spec.size()); ++r) {
        auto ro = [&](int c, const QString& s) { auto* it = new QTableWidgetItem(s); it->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable); t->setItem(r, c, it); };
        ro(0, spec[r].label); auto* v = new QTableWidgetItem; v->setBackground(spec[r].req ? kReq : kOpt); t->setItem(r, 1, v);
        if (reqCol) { ro(2, spec[r].unit); ro(3, spec[r].note); } else ro(2, spec[r].note);
    }
    return t;
}
QTableWidget* MainWindow::rowTable(const std::vector<ColSpec>& spec) {
    auto* t = new QTableWidget(0, static_cast<int>(spec.size()));
    for (int c = 0; c < static_cast<int>(spec.size()); ++c) {
        auto* h = new QTableWidgetItem(spec[c].head); h->setToolTip(spec[c].note); h->setData(Qt::UserRole, spec[c].req);
        if (spec[c].req) h->setForeground(QColor(140, 90, 0));
        t->setHorizontalHeaderItem(c, h); t->setColumnWidth(c, static_cast<int>(spec[c].width * 8.5));
    }
    t->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    t->horizontalHeader()->setFixedHeight(46);
    t->setWordWrap(true);
    return t;
}
void MainWindow::fillKv(QTableWidget* t, const std::vector<KvSpec>& spec, const std::map<std::string, std::string>& v) {
    for (int r = 0; r < static_cast<int>(spec.size()); ++r) { auto it = v.find(spec[r].label); t->item(r, 1)->setText(it == v.end() ? "" : Q(it->second)); }
}
std::map<std::string, std::string> MainWindow::readKv(QTableWidget* t, const std::vector<KvSpec>& spec) const {
    std::map<std::string, std::string> m;
    for (int r = 0; r < static_cast<int>(spec.size()); ++r) { const QString s = t->item(r, 1)->text().trimmed(); if (!s.isEmpty()) m[spec[r].label] = S(s); }
    return m;
}
void MainWindow::fillRows(QTableWidget* t, const std::vector<ColSpec>& spec, const std::vector<Row>& rows) {
    t->setRowCount(static_cast<int>(rows.size()));
    for (int r = 0; r < static_cast<int>(rows.size()); ++r)
        for (int c = 0; c < static_cast<int>(spec.size()); ++c) {
            auto f = rows[r].find(spec[c].head);
            auto* it = new QTableWidgetItem(f == rows[r].end() ? "" : Q(f->second)); it->setBackground(spec[c].req ? kReq : kOpt); t->setItem(r, c, it);
        }
}
std::vector<Row> MainWindow::readRows(QTableWidget* t, const std::vector<ColSpec>& spec) const {
    std::vector<Row> out;
    for (int r = 0; r < t->rowCount(); ++r) {
        Row x; bool any = false;
        for (int c = 0; c < static_cast<int>(spec.size()); ++c) { auto* it = t->item(r, c); const QString s = it ? it->text().trimmed() : QString(); if (!s.isEmpty()) { x[spec[c].head] = S(s); any = true; } }
        if (any) out.push_back(x);
    }
    return out;
}
QString MainWindow::hullVal(const char* label) const { const auto& sp = hullSpec(); for (int r = 0; r < static_cast<int>(sp.size()); ++r) if (std::string(sp[r].label) == label) return hull_->item(r, 1)->text().trimmed(); return {}; }
void MainWindow::setHullVal(const char* label, const QString& v) { const auto& sp = hullSpec(); for (int r = 0; r < static_cast<int>(sp.size()); ++r) if (std::string(sp[r].label) == label) hull_->item(r, 1)->setText(v); }

ShipDesign MainWindow::gather() const {
    ShipDesign d;
    d.hull = readKv(hull_, hullSpec()); d.thr = readRows(thr_, thrSpec()); d.sen = readRows(sen_, senSpec());
    d.outKv = readKv(outKv_, outKvSpec()); d.outRows = readRows(outRows_, outSpec());
    for (int v = 0; v < 3; ++v) { auto s = views_[v]->shapes(); d.shapes.insert(d.shapes.end(), s.begin(), s.end()); d.under[v] = views_[v]->underlay(); }
    return d;
}

// ---------------------------------------------------------------- виды
QWidget* MainWindow::buildView(int v) {
    auto* w = new QWidget; auto* l = new QHBoxLayout(w); l->setContentsMargins(4, 4, 4, 4);
    auto* tools = new QVBoxLayout; tools->setSpacing(4);
    auto* ve = new ViewEdit(v); views_[v] = ve;
    auto btn = [&](const QString& t, auto fn, const QString& tip = {}) { auto* b = new QPushButton(t); b->setToolTip(tip); connect(b, &QPushButton::clicked, this, fn); tools->addWidget(b); return b; };
    tools->addWidget(new QLabel("<b>Подложка</b>"));
    btn("Загрузить картинку…", [this, v] { const QString p = QFileDialog::getOpenFileName(this, "Фото или чертёж", QFileInfo(path_).absolutePath(), "Картинки (*.png *.jpg *.jpeg *.bmp *.gif *.tif *.tiff)"); if (!p.isEmpty()) loadUnderlay(v, p); });
    btn("Привязать (2 точки)", [ve] { ve->setTool(ViewEdit::Calib); }, "Щёлкните две точки с известными координатами");
    opac_[v] = new QSlider(Qt::Horizontal); opac_[v]->setRange(0, 100); opac_[v]->setValue(60);
    connect(opac_[v], &QSlider::valueChanged, this, [ve](int x) { ve->setOpacityValue(x / 100.0); });
    tools->addWidget(new QLabel("Прозрачность подложки")); tools->addWidget(opac_[v]);
    btn("Убрать подложку", [this, ve] { ve->setUnderlay(Underlay(), {}); setDirty(); });
    tools->addSpacing(8); tools->addWidget(new QLabel("<b>Фигуры</b>"));
    btn("Выбор и перемещение", [ve] { ve->setTool(ViewEdit::Select); }, "Перетаскивание фигур, вершин и меток; правая кнопка — сдвиг вида; колесо — масштаб");
    btn("Прямоугольник", [ve] { ve->setTool(ViewEdit::Rect); });
    btn("Многоугольник", [ve] { ve->setTool(ViewEdit::Poly); });
    btn("Удалить фигуру", [ve] { ve->deleteSelected(); });
    tools->addSpacing(8); tools->addWidget(new QLabel("<b>Распознавание</b>"));
    btn("Контур по картинке", [ve] { ve->setTool(ViewEdit::Contour); }, "Обведите рамкой предмет на подложке — программа предложит контур");
    btn("Распознать (Claude)", [this, v] { recognize(v); }, "Отправить подложку Claude и получить фигуры и подписанные размеры");
    tools->addSpacing(8);
    btn("Показать всё", [ve] { ve->fit(); });
    tools->addSpacing(8); tools->addWidget(new QLabel("<b>Схема по данным корпуса</b>"));
    auto* shOn = new QCheckBox("Показывать (зелёный пунктир)"); shOn->setChecked(true);
    connect(shOn, &QCheckBox::toggled, ve, &ViewEdit::setSchemeVisible); tools->addWidget(shOn);
    schemeNote_[v] = new QLabel; schemeNote_[v]->setWordWrap(true); schemeNote_[v]->setStyleSheet("color:#3c6e46"); tools->addWidget(schemeNote_[v]);
    btn("Сделать фигурами", [this, ve] {
        const auto s = ve->schemeShapes();
        if (s.empty()) { say("Схемы нет: заполните на вкладке «Корпус» длину, ширину, осадку, площади парусности."); return; }
        ve->addShapes(s); say(QString("Схема превращена в фигуры (%1). Их можно править; расчёт «По чертежу» теперь учитывает их.").arg(s.size()));
    }, "Превратить схему в обычные фигуры, чтобы править их");
    tools->addSpacing(8); tools->addWidget(new QLabel("<b>По чертежу</b>"));
    calc_[v] = new QLabel; calc_[v]->setWordWrap(true); calc_[v]->setTextFormat(Qt::RichText); calc_[v]->setMinimumWidth(230); tools->addWidget(calc_[v]);
    btn("Перенести в корпус", [this, v] { applyCalc(v); });
    tools->addStretch();
    auto* help = new QLabel(v == Top ? "Корпус сверху — для длины, ширины и полноты ватерлинии. Метки: <span style='color:#d07010'>●</span> движители, <span style='color:#20a0b0'>■</span> датчики."
                                     : v == Side ? "Всё, что выше синей линии (ватерлиния), — боковая парусность; ниже — подводная часть. Пунктир — осадка из данных корпуса."
                                                 : "Всё, что выше ватерлинии, — лобовая парусность.");
    help->setWordWrap(true); help->setStyleSheet("color:#555"); tools->addWidget(help);
    auto* tw = new QWidget; tw->setLayout(tools); tw->setFixedWidth(250);
    l->addWidget(tw); l->addWidget(ve, 1);
    connect(ve, &ViewEdit::changed, this, [this] { if (!loading_) setDirty(); refreshCalc(); });
    connect(ve, &ViewEdit::message, this, [this](const QString& m) { say(m); });
    connect(ve, &ViewEdit::cursorAt, this, [this, v](double a, double b) { coord_->setText(QString("%1  %2 = %3 м,  %4 = %5 м").arg(viewName(v)).arg(v == Front ? "Y" : "X").arg(a, 0, 'f', 2).arg(v == Top ? "Y" : "высота").arg(b, 0, 'f', 2)); });
    connect(ve, &ViewEdit::calibPicked, this, [this, v](QPointF a, QPointF b) { askCalibration(v, a, b); });
    connect(ve, &ViewEdit::contourPicked, this, [this, v](QRectF r) { runContour(v, r); });
    connect(ve, &ViewEdit::markerMoved, this, [this](const QString& id, bool t, double a, double b) { onMarkerMoved(id, t, a, b); });
    return w;
}

void MainWindow::loadUnderlay(int v, const QString& p) {
    QString err;
    if (!views_[v]->loadImage(p, err)) { QMessageBox::warning(this, "Подложка", err); return; }
    views_[v]->setOpacityValue(opac_[v]->value() / 100.0);
    views_[v]->fit(); setDirty();
    say("Подложка загружена. Привяжите её двумя точками: «Привязать (2 точки)».");
}

void MainWindow::askCalibration(int v, QPointF px1, QPointF px2) {
    QDialog d(this); d.setWindowTitle("Привязка подложки");
    auto* f = new QFormLayout(&d);
    const QString an = v == Front ? "Y" : "X", bn = v == Top ? "Y" : "высота над ватерлинией";
    double L = 0; numOf(hullVal("Длина по ватерлинии"), L);
    auto box = [&](double val) { auto* s = new QDoubleSpinBox; s->setRange(-2000, 2000); s->setDecimals(2); s->setSuffix(" м"); s->setValue(val); return s; };
    f->addRow(new QLabel(QString("Укажите координаты щёлкнутых точек (%1 — %2, %3).\nНапример, кормовой и носовой перпендикуляры на ватерлинии: X = −L/2 и +L/2, высота 0.")
                         .arg(an, v == Front ? "от ДП, + правый борт" : "от миделя, + к носу", bn)));
    auto *a1 = box(L > 0 && v != Front ? -L / 2 : 0), *b1 = box(0), *a2 = box(L > 0 && v != Front ? L / 2 : 10), *b2 = box(0);
    f->addRow("Точка 1: " + an, a1); f->addRow("Точка 1: " + bn, b1); f->addRow("Точка 2: " + an, a2); f->addRow("Точка 2: " + bn, b2);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel); f->addRow(bb);
    connect(bb, &QDialogButtonBox::accepted, &d, &QDialog::accept); connect(bb, &QDialogButtonBox::rejected, &d, &QDialog::reject);
    if (d.exec() != QDialog::Accepted) return;
    const QPointF m1(a1->value(), b1->value()), m2(a2->value(), b2->value());
    if (QLineF(m1, m2).length() < 0.01) { QMessageBox::warning(this, "Привязка", "Точки совпадают — привязка невозможна."); return; }
    calibrate(v, px1, px2, m1, m2);
}
void MainWindow::calibrate(int v, QPointF px1, QPointF px2, QPointF m1, QPointF m2) {
    views_[v]->setCalibration(px1, px2, m1, m2); views_[v]->fit();
    const double ppm = QLineF(px1, px2).length() / QLineF(m1, m2).length();
    say(QString("Подложка привязана: %1 пикселей на метр.").arg(ppm, 0, 'f', 1));
}

void MainWindow::runContour(int v, QRectF r) {
    ViewEdit* ve = views_[v];
    if (!ve->calibrated()) { say("Сначала привяжите подложку: без привязки размеры фигуры неизвестны."); return; }
    const QImage g = ve->image().convertToFormat(QImage::Format_Grayscale8);
    std::vector<uint8_t> buf(static_cast<size_t>(g.width()) * g.height());
    for (int y = 0; y < g.height(); ++y) std::memcpy(&buf[static_cast<size_t>(y) * g.width()], g.constScanLine(y), g.width());
    const auto res = dp::contour::find(buf, g.width(), g.height(), int(r.left()), int(r.top()), int(r.right()), int(r.bottom()));
    if (!res.ok) { say(QString("Контур не найден: ") + res.why + ". Обведите фигуру вручную."); return; }
    std::vector<QPointF> px; for (auto& p : res.pts) px.push_back({p.x, p.y});
    ve->addShapePx(px, "контур");
    say(QString("Предложен контур из %1 вершин. Проверьте его: вершины можно перетаскивать, двойной щелчок по стороне добавляет вершину.").arg(px.size()));
}

void MainWindow::refreshCalc() {
    const ShipDesign d = gather(); const DrawCalc c = calc(d);
    calc_[Top]->setText(c.top ? QString("Длина <b>%1 м</b><br>Ширина <b>%2 м</b><br>Площадь ватерлинии %3 м²<br>Полнота ватерлинии <b>%4</b>").arg(fmt(c.length, 2), fmt(c.beam, 2), fmt(c.waterplane, 1), fmt(c.cw, 3)) : "Нарисуйте корпус сверху.");
    QString s;
    if (c.side) s += QString("Боковая парусность <b>%1 м²</b><br>Центр от миделя <b>%2 м</b>, высота %3 м<br>").arg(fmt(c.lateral, 1), fmt(c.xWind, 2), fmt(c.zWind, 2));
    if (c.sideUnder) s += QString("Подводная площадь <b>%1 м²</b><br>Осадка по чертежу <b>%2 м</b>").arg(fmt(c.underwater, 1), fmt(c.draft, 2));
    calc_[Side]->setText(s.isEmpty() ? "Нарисуйте профиль: корпус и надстройки." : s);
    calc_[Front]->setText(c.front ? QString("Лобовая парусность <b>%1 м²</b>").arg(fmt(c.frontal, 1)) : "Нарисуйте вид спереди над водой.");
}
void MainWindow::applyCalc(int v) {
    const DrawCalc c = calc(gather()); QStringList done;
    auto put = [&](const char* k, double val, int dec) { setHullVal(k, fmt(val, dec)); done << QString("%1 = %2").arg(k, fmt(val, dec)); };
    if (v == Top && c.top) { put("Длина по ватерлинии", c.length, 2); put("Ширина", c.beam, 2); put("Коэффициент полноты ватерлинии", c.cw, 3); }
    if (v == Side) { if (c.side) { put("Боковая площадь парусности", c.lateral, 1); put("Центр парусности от миделя", c.xWind, 2); }
                     if (c.sideUnder) { put("Боковая площадь подводной части", c.underwater, 1); put("Осадка", c.draft, 2); } }
    if (v == Front && c.front) put("Лобовая площадь парусности", c.frontal, 1);
    if (done.isEmpty()) { say("Нечего переносить: на виде нет фигур."); return; }
    say("Перенесено в корпус: " + done.join("; "));
    refreshGuides(); refreshScheme();
}
void MainWindow::refreshGuides() {
    double T = 0, L = 0, B = 0; numOf(hullVal("Осадка"), T); numOf(hullVal("Длина по ватерлинии"), L); numOf(hullVal("Ширина"), B);
    for (auto* ve : views_) ve->setGuides(T, L, B);
}
void MainWindow::refreshScheme() {
    ShipDesign d; d.hull = readKv(hull_, hullSpec()); d.thr = readRows(thr_, thrSpec());
    const Scheme s = scheme(d);
    for (int v = 0; v < 3; ++v) {
        views_[v]->setScheme(s);
        int n = 0; for (auto& x : s.shapes) n += x.view == v ? 1 : 0;
        QString need;
        if (v == Top) need = "длину и ширину (полноту — по желанию)";
        else if (v == Side) need = "длину, осадку и боковую парусность (центр и подводную площадь — по желанию)";
        else need = "ширину, осадку и лобовую парусность";
        schemeNote_[v]->setText(n ? QString("Нарисовано по введённым числам. В расчёт «По чертежу» не входит.")
                                  : "Чтобы появилась схема, введите в «Корпус» " + need + ".");
    }
}
void MainWindow::refreshMarkers() {
    QVector<Marker> m;
    auto col = [](QTableWidget* t, const char* h) { for (int c = 0; c < t->columnCount(); ++c) if (t->horizontalHeaderItem(c)->text() == h) return c; return -1; };
    auto txt = [](QTableWidget* t, int r, int c) { auto* it = c >= 0 ? t->item(r, c) : nullptr; return it ? it->text().trimmed() : QString(); };
    for (QTableWidget* t : {thr_, sen_}) {
        const int ci = 0, cx = col(t, "X от миделя, м"), cy = col(t, "Y от ДП, м");
        for (int r = 0; r < t->rowCount(); ++r) {
            Marker k; k.id = txt(t, r, ci); k.thruster = t == thr_; double a, b;
            if (k.id.isEmpty() || !numOf(txt(t, r, cx), a) || !numOf(txt(t, r, cy), b)) continue;
            k.a = a; k.b = b; m.push_back(k);
        }
    }
    views_[Top]->setMarkers(m);
}
void MainWindow::onMarkerMoved(const QString& id, bool thr, double a, double b) {
    QTableWidget* t = thr ? thr_ : sen_;
    int cx = -1, cy = -1; for (int c = 0; c < t->columnCount(); ++c) { const QString h = t->horizontalHeaderItem(c)->text(); if (h == "X от миделя, м") cx = c; if (h == "Y от ДП, м") cy = c; }
    for (int r = 0; r < t->rowCount(); ++r) if (t->item(r, 0) && t->item(r, 0)->text().trimmed() == id) { t->item(r, cx)->setText(fmt(a, 1)); t->item(r, cy)->setText(fmt(b, 1)); }
    say(QString("%1: X = %2 м, Y = %3 м").arg(id, fmt(a, 1), fmt(b, 1)));
}

// ---------------------------------------------------------------- распознавание Claude
void MainWindow::claudeSettings() {
    QSettings st("SDP", "ShipEditor");
    QDialog d(this); d.setWindowTitle("Распознавание Claude");
    auto* f = new QFormLayout(&d);
    keystore::migrateFromSettings();
    f->addRow(new QLabel("Картинка подложки отправляется в Anthropic API (нужен интернет).\nКлюч хранится " + keystore::where() + ".\nЧтобы удалить ключ, очистите поле и нажмите «Сохранить»."));
    auto* key = new QLineEdit(keystore::load()); key->setEchoMode(QLineEdit::Password); key->setPlaceholderText("sk-ant-…");
    auto* model = new QLineEdit(st.value("model", "claude-sonnet-5-5").toString());
    f->addRow("Ключ API", key); f->addRow("Модель", model);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel); f->addRow(bb);
    connect(bb, &QDialogButtonBox::accepted, &d, &QDialog::accept); connect(bb, &QDialogButtonBox::rejected, &d, &QDialog::reject);
    if (d.exec() != QDialog::Accepted) return;
    if (!keystore::save(key->text().trimmed())) say("⚠ Ключ не удалось сохранить в хранилище системы: он будет действовать до закрытия программы.");
    st.setValue("model", model->text().trimmed().isEmpty() ? "claude-sonnet-5-5" : model->text().trimmed());
    say("Настройки распознавания сохранены.");
}
void MainWindow::recognize(int v) {
    ViewEdit* ve = views_[v];
    if (!ve->hasImage()) { say("Сначала загрузите подложку."); return; }
    QSettings st("SDP", "ShipEditor");
    keystore::migrateFromSettings();
    const QString key = keystore::load(), model = st.value("model", "claude-sonnet-5-5").toString();
    if (key.isEmpty()) { claudeSettings(); if (keystore::load().isEmpty()) return; recognize(v); return; }
    // уменьшение до 1568 точек по большей стороне (больше Claude всё равно не использует)
    const QImage& src = ve->image(); const double k = std::min(1.0, 1568.0 / std::max(src.width(), src.height()));
    const QImage img = k < 1 ? src.scaled(int(src.width() * k), int(src.height() * k), Qt::KeepAspectRatio, Qt::SmoothTransformation) : src;
    QByteArray bytes; QBuffer buf(&bytes); buf.open(QIODevice::WriteOnly); img.convertToFormat(QImage::Format_RGB32).save(&buf, "JPEG", 90);
    const std::string body = dp::vision::requestBody(model.toStdString(), v, img.width(), img.height(), "image/jpeg", bytes.toBase64().toStdString());
    const QByteArray testUrl = qgetenv("SDP_CLAUDE_URL");   // только для проверки программы на подставном сервере
    QNetworkRequest rq(QUrl(testUrl.isEmpty() ? QString("https://api.anthropic.com/v1/messages") : QString::fromUtf8(testUrl)));
    rq.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    rq.setRawHeader("x-api-key", key.toUtf8()); rq.setRawHeader("anthropic-version", "2023-06-01");
    rq.setTransferTimeout(180000);
    say("Картинка отправлена на распознавание, ждём ответ (обычно 10–60 с)…");
    QApplication::setOverrideCursor(Qt::BusyCursor);
    QNetworkReply* rp = net_->post(rq, QByteArray::fromStdString(body));
    connect(rp, &QNetworkReply::finished, this, [this, rp, v, k] {
        QApplication::restoreOverrideCursor(); rp->deleteLater();
        const QByteArray data = rp->readAll();
        if (rp->error() != QNetworkReply::NoError && data.isEmpty()) { say("Распознавание: нет ответа: " + rp->errorString()); if (!quietDialogs) QMessageBox::warning(this, "Распознавание", "Нет ответа: " + rp->errorString()); return; }
        const dp::vision::Answer a = dp::vision::parseResponse(data.toStdString());
        if (!a.ok) { say("Распознавание: " + Q(a.error)); if (!quietDialogs) QMessageBox::warning(this, "Распознавание", Q(a.error)); return; }
        ViewEdit* ve = views_[v]; int placed = 0;
        if (ve->calibrated()) for (const auto& f : a.figures) { std::vector<QPointF> px; for (auto& p : f.px) px.push_back({p.first / k, p.second / k}); ve->addShapePx(px, Q(f.name)); ++placed; }
        QString msg = QString("Claude предложил фигур: %1").arg(a.figures.size());
        if (!ve->calibrated() && !a.figures.empty()) msg += " — не размещены: подложка не привязана. Привяжите и повторите.";
        else msg += QString(", размещено: %1. Проверьте и поправьте их.").arg(placed);
        if (!a.notes.empty()) msg += "\nЗамечание: " + Q(a.notes);
        if (!a.dims.empty()) {
            QDialog d(this); d.setWindowTitle("Размеры с чертежа"); auto* l = new QVBoxLayout(&d);
            l->addWidget(new QLabel(msg + "\n\nНа чертеже прочитаны размеры. Отметьте, что перенести в корпус:"));
            std::vector<std::pair<QCheckBox*, std::pair<std::string, double>>> boxes;
            for (auto& dm : a.dims) { auto* c = new QCheckBox(QString("%1: %2 (сейчас: %3)").arg(Q(dm.first), fmt(dm.second, 2), hullVal(dm.first.c_str()).isEmpty() ? "пусто" : hullVal(dm.first.c_str()))); c->setChecked(hullVal(dm.first.c_str()).isEmpty()); l->addWidget(c); boxes.push_back({c, dm}); }
            auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel); l->addWidget(bb);
            connect(bb, &QDialogButtonBox::accepted, &d, &QDialog::accept); connect(bb, &QDialogButtonBox::rejected, &d, &QDialog::reject);
            if (quietDialogs || d.exec() == QDialog::Accepted) for (auto& b : boxes) if (b.first->isChecked()) setHullVal(b.second.first.c_str(), fmt(b.second.second, 2));
            refreshGuides(); refreshScheme();
        } else if (!quietDialogs) QMessageBox::information(this, "Распознавание", msg);
        say(msg.section('\n', 0, 0));
    });
}

// ---------------------------------------------------------------- файлы
void MainWindow::newShip() {
    loading_ = true;
    ShipDesign d; fillKv(hull_, hullSpec(), d.hull); fillRows(thr_, thrSpec(), {}); fillRows(sen_, senSpec(), {}); fillKv(outKv_, outKvSpec(), {}); fillRows(outRows_, outSpec(), {});
    for (int v = 0; v < 3; ++v) { views_[v]->setShapes({}); views_[v]->setUnderlay(Underlay(), {}); opac_[v]->setValue(60); }
    path_.clear(); loading_ = false; dirty_ = false;
    refreshCalc(); refreshMarkers(); refreshGuides(); refreshScheme(); for (auto* ve : views_) ve->fit();
    setWindowTitle("Данные судна — новое судно");
}
bool MainWindow::openBook(const QString& p) {
    ShipDesign d; std::string err; std::vector<std::string> notes;
    if (!readBook(p.toStdString(), d, err, &notes)) { QMessageBox::warning(this, "Открыть книгу", Q(err)); return false; }
    loading_ = true;
    fillKv(hull_, hullSpec(), d.hull); fillRows(thr_, thrSpec(), d.thr); fillRows(sen_, senSpec(), d.sen); fillKv(outKv_, outKvSpec(), d.outKv); fillRows(outRows_, outSpec(), d.outRows);
    const QString dir = QFileInfo(p).absolutePath();
    for (int v = 0; v < 3; ++v) { views_[v]->setShapes(d.shapes); views_[v]->setUnderlay(d.under[v], dir); opac_[v]->setValue(int(std::lround(d.under[v].opacity * 100))); }
    loading_ = false; path_ = QFileInfo(p).absoluteFilePath(); dirty_ = false;
    refreshCalc(); refreshMarkers(); refreshGuides(); refreshScheme(); for (auto* ve : views_) ve->fit();
    setWindowTitle("Данные судна — " + QFileInfo(p).fileName());
    say("Открыта книга " + QFileInfo(p).fileName() + (notes.empty() ? QString() : ". " + Q(notes.front())));
    return true;
}
bool MainWindow::saveBook(const QString& p) {
    ShipDesign d = gather();
    const QDir dir = QFileInfo(p).absoluteDir();
    for (auto& u : d.under) if (!u.path.empty()) u.path = dir.relativeFilePath(Q(u.path)).toStdString();   // пути подложек — относительно книги
    std::string err;
    if (!writeBook(d, p.toStdString(), err)) { QMessageBox::warning(this, "Сохранить", Q(err)); return false; }
    path_ = QFileInfo(p).absoluteFilePath(); dirty_ = false;
    setWindowTitle("Данные судна — " + QFileInfo(p).fileName());
    say("Сохранено: " + QFileInfo(p).fileName() + ". Картинки подложек не копируются в книгу — храните их рядом с книгой.");
    return true;
}
void MainWindow::checkBook() {
    QTemporaryDir td; const QString p = td.filePath("проверка.xlsx"); std::string err;
    if (!writeBook(gather(), p.toStdString(), err)) { report_->setPlainText("Не удалось подготовить книгу для проверки: " + Q(err)); return; }
    dp::AppCfg cfg; dp::BookReport rep; const bool ok = dp::loadShipBook(p.toStdString(), cfg, rep);
    QString s = ok ? "КНИГА В ПОРЯДКЕ: ядро примет это судно.\n\n" : QString("ЕСТЬ ОШИБКИ: %1. Ядро не загрузит судно, пока они не исправлены.\n\n").arg(rep.errors.size());
    for (auto& e : rep.errors) s += "ОШИБКА: " + Q(e) + "\n";
    for (auto& w : rep.warnings) s += "Предупреждение: " + Q(w) + "\n";
    if (!rep.defaults.empty()) s += "\n";
    for (auto& x : rep.defaults) s += "По умолчанию: " + Q(x) + "\n";
    if (!rep.estimates.empty()) s += "\n";
    for (auto& e : rep.estimates) s += "Оценка: " + Q(e) + "\n";
    report_->setPlainText(s); tabs_->setCurrentIndex(tabs_->count() - 1);
    say(ok ? "Проверка: книга в порядке." : QString("Проверка: ошибок %1.").arg(rep.errors.size()));
}
void MainWindow::setTab(int i) { tabs_->setCurrentIndex(i); }
void MainWindow::setDirty(bool d) { dirty_ = d; if (d && !windowTitle().endsWith("*")) setWindowTitle(windowTitle() + " *"); }
bool MainWindow::confirmDiscard() {
    if (!dirty_) return true;
    return QMessageBox::question(this, "Несохранённые изменения", "Изменения не сохранены. Продолжить без сохранения?") == QMessageBox::Yes;
}
void MainWindow::closeEvent(QCloseEvent* e) { if (confirmDiscard()) e->accept(); else e->ignore(); }
void MainWindow::say(const QString& s) { statusBar()->showMessage(s, 15000); }
