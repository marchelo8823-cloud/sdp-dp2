#include "viewedit.h"

#include <QApplication>
#include <QFileInfo>
#include <QDir>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QImageReader>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QScrollBar>
#include <QStyleOptionGraphicsItem>
#include <QWheelEvent>
#include <cmath>
#include <complex>

using namespace dp::design;

// фигура без стандартной пунктирной рамки выделения Qt: выделение показывается цветом и квадратиками вершин
class ShapeItem : public QGraphicsPolygonItem {
public:
    using QGraphicsPolygonItem::QGraphicsPolygonItem;
    void paint(QPainter* p, const QStyleOptionGraphicsItem* o, QWidget* w) override { QStyleOptionGraphicsItem x(*o); x.state &= ~QStyle::State_Selected; QGraphicsPolygonItem::paint(p, &x, w); }
};

static const QColor kShape(40, 120, 200), kShapeSel(230, 120, 20), kGrid(220, 226, 232), kGrid5(195, 204, 214), kAxis(90, 100, 110), kWater(20, 110, 200);

ViewEdit::ViewEdit(int view, QWidget* parent) : QGraphicsView(parent), view_(view), sc_(new QGraphicsScene(this)) {
    setScene(sc_);
    sc_->setSceneRect(-1000, -1000, 2000, 2000);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(QGraphicsView::NoAnchor);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setMouseTracking(true);
    setDragMode(QGraphicsView::NoDrag);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setBackgroundBrush(Qt::white);
    setFocusPolicy(Qt::StrongFocus);
    scale(8, 8);
    handles_ = sc_->addPath(QPainterPath(), QPen(kShapeSel, 0), QBrush(Qt::white)); handles_->setZValue(20);
    polyPreview_ = sc_->addPath(QPainterPath(), QPen(kShapeSel, 0, Qt::DashLine)); polyPreview_->setZValue(19);
    rubber_ = sc_->addRect(QRectF(), QPen(kShapeSel, 0, Qt::DashLine), QBrush(QColor(230, 120, 20, 30))); rubber_->setZValue(19); rubber_->hide();
}

// ---------------------------------------------------------------- фигуры
void ViewEdit::styleShape(QGraphicsPolygonItem* it, bool sel) {
    const QColor c = sel ? kShapeSel : kShape;
    QPen pen(c, 0); pen.setCosmetic(true); pen.setWidthF(sel ? 2.0 : 1.5);
    it->setPen(pen); it->setBrush(QColor(c.red(), c.green(), c.blue(), sel ? 60 : 45));
}
QGraphicsPolygonItem* ViewEdit::makeShape(const QPolygonF& poly, const QString& name) {
    auto* it = new ShapeItem(poly); sc_->addItem(it); it->setZValue(5); it->setData(0, name); it->setFlag(QGraphicsItem::ItemIsSelectable);
    styleShape(it, false); items_.push_back(it); return it;
}
void ViewEdit::setShapes(const std::vector<dp::design::Shape>& s) {
    for (auto* it : items_) { sc_->removeItem(it); delete it; }
    items_.clear();
    for (const auto& x : s) {
        if (x.view != view_) continue;
        QPolygonF p; for (const auto& q : x.pts) p << toScene(q.a, q.b);
        makeShape(p, QString::fromStdString(x.name));
    }
    updateHandles(); viewport()->update();
}
std::vector<dp::design::Shape> ViewEdit::shapes() const {
    std::vector<dp::design::Shape> r;
    for (auto* it : items_) {
        dp::design::Shape s; s.view = view_; s.name = it->data(0).toString().toStdString();
        for (const QPointF& q : it->polygon()) { Pt p; fromScene(q, p.a, p.b); p.a = std::round(p.a * 1000) / 1000; p.b = std::round(p.b * 1000) / 1000; s.pts.push_back(p); }
        r.push_back(s);
    }
    return r;
}
void ViewEdit::addShapePx(const std::vector<QPointF>& px, const QString& name) {
    if (!pix_) return;
    QPolygonF p; for (const QPointF& q : px) p << pix_->mapToScene(q);
    sc_->clearSelection();
    auto* it = makeShape(p, name); it->setSelected(true); updateHandles();
    emit changed();
}
QGraphicsPolygonItem* ViewEdit::selectedShape() const { for (auto* it : items_) if (it->isSelected()) return it; return nullptr; }
void ViewEdit::deleteSelected() {
    auto* it = selectedShape(); if (!it) { emit message("Не выбрана фигура."); return; }
    items_.erase(std::remove(items_.begin(), items_.end(), it), items_.end()); sc_->removeItem(it); delete it;
    updateHandles(); emit changed();
}
void ViewEdit::updateHandles() {
    QPainterPath path;
    if (auto* it = selectedShape()) {
        const double r = 4.0 / transform().m11();
        for (const QPointF& q : it->polygon()) path.addRect(QRectF(q.x() - r, q.y() - r, 2 * r, 2 * r));
    }
    handles_->setPath(path);
    for (auto* it : items_) styleShape(it, it->isSelected());
}
int ViewEdit::vertexAt(QGraphicsPolygonItem* it, QPoint vp) const {
    if (!it) return -1;
    const QPolygonF p = it->polygon();
    for (int i = 0; i < p.size(); ++i) { const QPoint q = mapFromScene(p[i]); if ((q - vp).manhattanLength() <= 8) return i; }
    return -1;
}

// ---------------------------------------------------------------- подложка
bool ViewEdit::loadImage(const QString& path, QString& err) {
    QImageReader rd(path); rd.setAutoTransform(true);
    QImage im = rd.read();
    if (im.isNull()) { err = "картинка не прочитана: " + rd.errorString() + " (поддерживаются PNG, JPG, BMP, GIF, TIFF; PDF сохраните как PNG)"; return false; }
    img_ = im.convertToFormat(QImage::Format_ARGB32); imgPath_ = QFileInfo(path).absoluteFilePath();
    if (!pix_) { pix_ = sc_->addPixmap(QPixmap()); pix_->setZValue(-10); pix_->setTransformationMode(Qt::SmoothTransformation); }
    pix_->setPixmap(QPixmap::fromImage(img_)); pix_->setOpacity(u_.opacity);
    u_.path = imgPath_.toStdString();
    applyUnderlayTransform();
    return true;
}
void ViewEdit::setUnderlay(const dp::design::Underlay& u, const QString& baseDir) {
    u_ = u;
    if (pix_) { sc_->removeItem(pix_); delete pix_; pix_ = nullptr; img_ = QImage(); imgPath_.clear(); }
    if (u.path.empty()) { viewport()->update(); return; }
    QString p = QString::fromStdString(u.path);
    if (QFileInfo(p).isRelative()) p = QDir(baseDir).filePath(p);
    QString err;
    if (!loadImage(p, err)) { emit message("Подложка «" + p + "» не загружена: " + err); u_.path.clear(); }
    else u_ = u, u_.path = imgPath_.toStdString(), applyUnderlayTransform();
}
void ViewEdit::setCalibration(QPointF px1, QPointF px2, QPointF m1, QPointF m2) {
    u_.cal = true; u_.px1 = px1.x(); u_.py1 = px1.y(); u_.px2 = px2.x(); u_.py2 = px2.y(); u_.a1 = m1.x(); u_.b1 = m1.y(); u_.a2 = m2.x(); u_.b2 = m2.y();
    applyUnderlayTransform(); emit changed();
}
void ViewEdit::setOpacityValue(double o) { u_.opacity = o; if (pix_) pix_->setOpacity(o); }
// подобие: пиксели → сцена по двум точкам (поворот, масштаб, сдвиг)
void ViewEdit::applyUnderlayTransform() {
    if (!pix_) return;
    if (!u_.cal) {   // без привязки: картинка шириной 100 м с центром в начале координат
        const double s = 100.0 / std::max(1, img_.width());
        pix_->setTransform(QTransform(s, 0, 0, s, -50.0, -img_.height() * s / 2));
        return;
    }
    using C = std::complex<double>;
    const C p1(u_.px1, u_.py1), p2(u_.px2, u_.py2);
    const QPointF s1 = toScene(u_.a1, u_.b1), s2 = toScene(u_.a2, u_.b2);
    const C q1(s1.x(), s1.y()), q2(s2.x(), s2.y());
    if (std::abs(p2 - p1) < 1e-9) return;
    const C z = (q2 - q1) / (p2 - p1), t = q1 - z * p1;
    pix_->setTransform(QTransform(z.real(), z.imag(), -z.imag(), z.real(), t.real(), t.imag()));
}
QPointF ViewEdit::sceneToPx(QPointF s) const { return pix_ ? pix_->mapFromScene(s) : QPointF(); }

// ---------------------------------------------------------------- метки
void ViewEdit::setMarkers(const QVector<Marker>& m) {
    if (drag_ == MoveMarker) return;
    for (auto* it : markerItems_) { sc_->removeItem(it); delete it; }
    markerItems_.clear(); markers_ = m;
    for (const auto& k : m) {
        const QPointF c = toScene(k.a, k.b);
        QGraphicsItem* g;
        if (k.thruster) { auto* e = sc_->addEllipse(-1, -1, 2, 2, QPen(QColor(150, 60, 0), 0), QBrush(QColor(240, 140, 40))); g = e; }
        else { auto* r = sc_->addRect(-0.7, -0.7, 1.4, 1.4, QPen(QColor(0, 90, 110), 0), QBrush(QColor(40, 190, 210))); g = r; }
        g->setFlag(QGraphicsItem::ItemIgnoresTransformations); g->setPos(c); g->setZValue(15);
        g->setScale(5);
        auto* t = new QGraphicsSimpleTextItem(k.id, g); t->setScale(0.2); t->setPos(1.3, -1.6); t->setBrush(QColor(40, 40, 40));
        markerItems_.push_back(g);
    }
    layoutLabels();
}

// Подпись ставится справа от метки; если место занято другой подписью или меткой — выше или ниже, ступенями по высоте строки.
void ViewEdit::layoutLabels() {
    std::vector<QRectF> busy;
    for (auto* g : markerItems_) { const QPointF c = mapFromScene(g->pos()); busy.push_back(QRectF(c - QPointF(5, 5), QSizeF(10, 10))); }
    for (auto* g : markerItems_) {
        if (g->childItems().isEmpty()) continue;
        auto* t = g->childItems().first();
        const double k = g->scale() * t->scale();                       // пикселей экрана на единицу текста
        const QRectF b = t->boundingRect();
        const QSizeF sz(b.width() * k, b.height() * k);
        const QPointF c = mapFromScene(g->pos());
        const double step = sz.height() + 1;
        QPointF best(6.5, -8);
        for (int i = 0; i < 12; ++i) {
            const double dy = -8 + ((i + 1) / 2) * step * (i % 2 ? -1 : 1);
            const QRectF r(c + QPointF(6.5, dy), sz);
            bool hit = false;
            for (const auto& q : busy) if (q.intersects(r)) { hit = true; break; }
            if (!hit) { best = QPointF(6.5, dy); break; }
        }
        busy.push_back(QRectF(c + best, sz));
        t->setPos(best / g->scale());
    }
}

// ---------------------------------------------------------------- схема по данным корпуса
void ViewEdit::setScheme(const Scheme& s) {
    for (auto* it : schemeItems_) { sc_->removeItem(it); delete it; }
    schemeItems_.clear(); scheme_.clear();
    for (const auto& x : s.shapes) {
        if (x.view != view_) continue;
        scheme_.push_back(x);
        QPolygonF p; for (const auto& q : x.pts) p << toScene(q.a, q.b);
        QPen pen(QColor(70, 140, 90), 0, Qt::DashLine); pen.setCosmetic(true); pen.setWidthF(1.6);
        auto* it = sc_->addPolygon(p, pen, QBrush(QColor(70, 140, 90, 22))); it->setZValue(2); it->setVisible(schemeOn_);
        it->setToolTip(QString::fromStdString(x.name) + " (схема по данным корпуса)");
        schemeItems_.push_back(it);
    }
    for (const auto& k : s.points) {
        if (k.view != view_) continue;
        const QPointF c = toScene(k.a, k.b);
        QGraphicsItem* g;
        if (k.thruster) { auto* e = sc_->addEllipse(-1, -1, 2, 2, QPen(QColor(150, 60, 0), 0, Qt::DashLine), QBrush(QColor(240, 140, 40, 90))); g = e; }
        else { QPainterPath pp; pp.moveTo(-1.2, 0); pp.lineTo(1.2, 0); pp.moveTo(0, -1.2); pp.lineTo(0, 1.2); pp.addEllipse(QPointF(0, 0), 0.6, 0.6); g = sc_->addPath(pp, QPen(QColor(70, 140, 90), 0)); }
        g->setFlag(QGraphicsItem::ItemIgnoresTransformations); g->setPos(c); g->setZValue(3); g->setScale(5); g->setVisible(schemeOn_);
        auto* t = new QGraphicsSimpleTextItem(QString::fromStdString(k.label), g); t->setScale(0.2); t->setPos(1.3, -1.6); t->setBrush(QColor(60, 110, 70));
        schemeItems_.push_back(g);
    }
    viewport()->update();
}
void ViewEdit::setSchemeVisible(bool on) { schemeOn_ = on; for (auto* it : schemeItems_) it->setVisible(on); viewport()->update(); }
std::vector<dp::design::Shape> ViewEdit::schemeShapes() const { return scheme_; }
void ViewEdit::addShapes(const std::vector<dp::design::Shape>& s) {
    sc_->clearSelection();
    for (const auto& x : s) { if (x.view != view_) continue; QPolygonF p; for (const auto& q : x.pts) p << toScene(q.a, q.b); makeShape(p, QString::fromStdString(x.name)); }
    updateHandles(); emit changed();
}

// ---------------------------------------------------------------- инструменты и мышь
void ViewEdit::setTool(Tool t) {
    cancelDraw(); tool_ = t;
    setCursor(t == Select ? Qt::ArrowCursor : Qt::CrossCursor);
    if (t == Calib) emit message(hasImage() ? "Привязка: щёлкните две точки на подложке, расстояние между которыми известно." : "Сначала загрузите подложку.");
    if (t == Contour) emit message(hasImage() ? "Контур: обведите рамкой предмет на подложке." : "Сначала загрузите подложку.");
    if (t == Poly) emit message("Многоугольник: щёлкайте вершины; двойной щелчок или Enter — закончить, Esc — отменить.");
    if (t == Rect) emit message("Прямоугольник: протяните рамку.");
}
void ViewEdit::cancelDraw() { poly_.clear(); polyPreview_->setPath(QPainterPath()); rubber_->hide(); calibPx_.clear(); drag_ = None; }
void ViewEdit::finishPoly() {
    if (poly_.size() >= 3) { sc_->clearSelection(); auto* it = makeShape(poly_, ""); it->setSelected(true); emit changed(); }
    poly_.clear(); polyPreview_->setPath(QPainterPath()); updateHandles();
}

void ViewEdit::mousePressEvent(QMouseEvent* e) {
    setFocus();
    lastVp_ = e->pos(); const QPointF s = mapToScene(e->pos()); startSc_ = s;
    if (e->button() == Qt::RightButton || e->button() == Qt::MiddleButton) { drag_ = Pan; setCursor(Qt::ClosedHandCursor); return; }
    if (e->button() != Qt::LeftButton) return;
    switch (tool_) {
        case Select: {
            if (auto* it = selectedShape()) { vtx_ = vertexAt(it, e->pos()); if (vtx_ >= 0) { drag_ = MoveVertex; return; } }
            for (int i = 0; i < static_cast<int>(markerItems_.size()); ++i) {
                const QPoint q = mapFromScene(markerItems_[i]->pos());
                if ((q - e->pos()).manhattanLength() <= 10) { marker_ = i; drag_ = MoveMarker; return; }
            }
            QGraphicsPolygonItem* hit = nullptr;
            for (auto it = items_.rbegin(); it != items_.rend(); ++it) if ((*it)->polygon().containsPoint(s, Qt::OddEvenFill)) { hit = *it; break; }
            sc_->clearSelection();
            if (hit) { hit->setSelected(true); drag_ = MoveShape; }
            else drag_ = Pan;
            updateHandles();
            return;
        }
        case Rect: drag_ = DrawRect; rubber_->setRect(QRectF(s, s)); rubber_->show(); return;
        case Contour: if (!hasImage()) { emit message("Сначала загрузите подложку."); return; } drag_ = PickRect; rubber_->setRect(QRectF(s, s)); rubber_->show(); return;
        case Poly: {
            if (poly_.size() >= 3 && (mapFromScene(poly_.first()) - e->pos()).manhattanLength() <= 8) { finishPoly(); return; }
            poly_ << s; return;
        }
        case Calib: {
            if (!hasImage()) { emit message("Сначала загрузите подложку."); return; }
            calibPx_.push_back(sceneToPx(s));
            QPainterPath pp; for (const QPointF& q : calibPx_) { const QPointF c = pix_->mapToScene(q); const double r = 5.0 / transform().m11(); pp.addEllipse(c, r, r); }
            polyPreview_->setPath(pp);
            if (calibPx_.size() == 1) emit message("Привязка: щёлкните вторую точку.");
            if (calibPx_.size() == 2) { const QPointF a = calibPx_[0], b = calibPx_[1]; cancelDraw(); setTool(Select); emit calibPicked(a, b); }
            return;
        }
    }
}
void ViewEdit::mouseMoveEvent(QMouseEvent* e) {
    const QPointF s = mapToScene(e->pos());
    double a, b; fromScene(s, a, b); emit cursorAt(a, b);
    const QPoint d = e->pos() - lastVp_; lastVp_ = e->pos();
    switch (drag_) {
        case Pan: translate(d.x() / transform().m11(), d.y() / transform().m22()); viewport()->update(); return;
        case MoveShape: if (auto* it = selectedShape()) { const QPointF dd = s - startSc_; startSc_ = s; it->setPolygon(it->polygon().translated(dd)); updateHandles(); } return;
        case MoveVertex: if (auto* it = selectedShape()) { QPolygonF p = it->polygon(); if (vtx_ < p.size()) { p[vtx_] = s; it->setPolygon(p); updateHandles(); } } return;
        case MoveMarker: if (marker_ >= 0) markerItems_[marker_]->setPos(s); return;
        case DrawRect: case PickRect: rubber_->setRect(QRectF(startSc_, s).normalized()); return;
        default: break;
    }
    if (tool_ == Poly && !poly_.isEmpty()) { QPainterPath pp; pp.addPolygon(poly_); pp.lineTo(s); polyPreview_->setPath(pp); }
}
void ViewEdit::mouseReleaseEvent(QMouseEvent* e) {
    const QPointF s = mapToScene(e->pos());
    const Drag was = drag_; drag_ = None;
    if (tool_ == Select) setCursor(Qt::ArrowCursor);
    if (was == MoveShape || was == MoveVertex) { if ((s - startSc_).manhattanLength() > 0 || was == MoveVertex) emit changed(); }
    if (was == MoveMarker && marker_ >= 0) {
        double a, b; fromScene(markerItems_[marker_]->pos(), a, b);
        a = std::round(a * 10) / 10; b = std::round(b * 10) / 10;
        const Marker m = markers_[marker_]; marker_ = -1;
        emit markerMoved(m.id, m.thruster, a, b);
    }
    if (was == DrawRect) {
        rubber_->hide(); const QRectF r = QRectF(startSc_, s).normalized();
        if (mapFromScene(r.topLeft()).x() + 4 < mapFromScene(r.bottomRight()).x()) {
            sc_->clearSelection(); auto* it = makeShape(QPolygonF(r), ""); it->setSelected(true); updateHandles(); emit changed();
        }
    }
    if (was == PickRect) {
        rubber_->hide(); const QRectF r = QRectF(startSc_, s).normalized();
        QPolygonF pp; pp << sceneToPx(r.topLeft()) << sceneToPx(r.topRight()) << sceneToPx(r.bottomRight()) << sceneToPx(r.bottomLeft());
        setTool(Select); emit contourPicked(pp.boundingRect());
    }
}
void ViewEdit::mouseDoubleClickEvent(QMouseEvent* e) {
    if (tool_ == Poly) { if (!poly_.isEmpty()) poly_.removeLast(); finishPoly(); return; }
    if (tool_ != Select) return;
    auto* it = selectedShape(); if (!it) return;
    const QPointF s = mapToScene(e->pos()); QPolygonF p = it->polygon();
    // вставка вершины на ближайшую сторону
    int best = -1; double bd = 1e18;
    for (int i = 0; i < p.size(); ++i) {
        const QPointF A = p[i], B = p[(i + 1) % p.size()]; const QPointF AB = B - A; const double L2 = QPointF::dotProduct(AB, AB);
        double t = L2 > 0 ? QPointF::dotProduct(s - A, AB) / L2 : 0; t = std::clamp(t, 0.0, 1.0);
        const QPointF q = A + t * AB; const double dd = QLineF(q, s).length();
        if (dd < bd) { bd = dd; best = i; }
    }
    if (best >= 0 && bd * transform().m11() < 10) { p.insert(best + 1, s); it->setPolygon(p); updateHandles(); emit changed(); emit message("Вершина добавлена. Удалить вершину: щёлкнуть по ней с Alt."); }
}
void ViewEdit::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Escape) { cancelDraw(); setTool(Select); return; }
    if ((e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter) && tool_ == Poly) { finishPoly(); return; }
    if (e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) { deleteSelected(); return; }
    QGraphicsView::keyPressEvent(e);
}
void ViewEdit::wheelEvent(QWheelEvent* e) {
    const double f = std::pow(1.0015, e->angleDelta().y());
    const QPointF before = mapToScene(e->position().toPoint());
    scale(f, f);
    const QPointF after = mapToScene(e->position().toPoint());
    translate(after.x() - before.x(), after.y() - before.y());
    updateHandles();
    layoutLabels();
}
void ViewEdit::fit() {
    QRectF r;
    for (auto* it : items_) r = r.united(it->sceneBoundingRect());
    if (pix_) r = r.united(pix_->sceneBoundingRect());
    for (auto* m : markerItems_) r = r.united(QRectF(m->pos() - QPointF(1, 1), QSizeF(2, 2)));
    if (schemeOn_) for (auto* m : schemeItems_) r = r.united(m->flags() & QGraphicsItem::ItemIgnoresTransformations ? QRectF(m->pos() - QPointF(1, 1), QSizeF(2, 2)) : m->sceneBoundingRect());
    if (len_ > 0) r = r.united(QRectF(toScene(-len_ / 2, view_ == Top ? -beam_ / 2 : 0), toScene(len_ / 2, view_ == Top ? beam_ / 2 : 0)).normalized());
    if (r.isEmpty()) r = QRectF(-40, -20, 80, 40);
    r.adjust(-r.width() * 0.08 - 2, -r.height() * 0.12 - 2, r.width() * 0.08 + 2, r.height() * 0.12 + 2);
    fitInView(r, Qt::KeepAspectRatio);
    updateHandles();
    layoutLabels();
}

// ---------------------------------------------------------------- сетка и подписи
static double niceStep(double minM) { const double s[] = {0.5, 1, 2, 5, 10, 20, 50, 100, 200}; for (double x : s) if (x >= minM) return x; return 500; }
void ViewEdit::drawBackground(QPainter* p, const QRectF& r) {
    p->fillRect(r, Qt::white);
    const double k = transform().m11(), step = niceStep(14.0 / k);
    QPen pen(kGrid, 0); pen.setCosmetic(true);
    auto lines = [&](double st, const QColor& c) {
        QPen q(c, 0); q.setCosmetic(true); p->setPen(q);
        for (double x = std::floor(r.left() / st) * st; x <= r.right(); x += st) p->drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
        for (double y = std::floor(r.top() / st) * st; y <= r.bottom(); y += st) p->drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
    };
    lines(step, kGrid); lines(step * 5, kGrid5);
    QPen ax(kAxis, 0); ax.setCosmetic(true); ax.setWidthF(1.2); p->setPen(ax);
    p->drawLine(QPointF(0, r.top()), QPointF(0, r.bottom()));
    if (view_ == Top) p->drawLine(QPointF(r.left(), 0), QPointF(r.right(), 0));
    else {
        QPen w(kWater, 0); w.setCosmetic(true); w.setWidthF(1.8); p->setPen(w); p->drawLine(QPointF(r.left(), 0), QPointF(r.right(), 0));
        p->fillRect(QRectF(QPointF(r.left(), 0), QPointF(r.right(), r.bottom())), QColor(20, 110, 200, 14));
        if (draft_ > 0) { QPen d(kWater, 0, Qt::DashLine); d.setCosmetic(true); p->setPen(d); p->drawLine(QPointF(r.left(), draft_), QPointF(r.right(), draft_)); }
    }
    if (len_ > 0 && view_ != Front) {   // перпендикуляры по длине из данных корпуса
        QPen d(QColor(150, 150, 150), 0, Qt::DotLine); d.setCosmetic(true); p->setPen(d);
        p->drawLine(QPointF(-len_ / 2, r.top()), QPointF(-len_ / 2, r.bottom())); p->drawLine(QPointF(len_ / 2, r.top()), QPointF(len_ / 2, r.bottom()));
    }
    if (beam_ > 0 && view_ != Side) {
        QPen d(QColor(150, 150, 150), 0, Qt::DotLine); d.setCosmetic(true); p->setPen(d);
        if (view_ == Top) { p->drawLine(QPointF(r.left(), -beam_ / 2), QPointF(r.right(), -beam_ / 2)); p->drawLine(QPointF(r.left(), beam_ / 2), QPointF(r.right(), beam_ / 2)); }
        else { p->drawLine(QPointF(-beam_ / 2, r.top()), QPointF(-beam_ / 2, r.bottom())); p->drawLine(QPointF(beam_ / 2, r.top()), QPointF(beam_ / 2, r.bottom())); }
    }
}
void ViewEdit::drawForeground(QPainter* p, const QRectF&) {
    p->save(); p->resetTransform();
    const QRect vr = viewport()->rect(); const double k = transform().m11(), step = niceStep(60.0 / k) ;
    QFont f = font(); f.setPointSizeF(8.5); p->setFont(f); p->setPen(QColor(90, 90, 90));
    const QPointF tl = mapToScene(vr.topLeft()), br = mapToScene(vr.bottomRight());
    for (double x = std::ceil(tl.x() / step) * step; x <= br.x(); x += step) { const QPoint q = mapFromScene(QPointF(x, 0)); p->drawText(q.x() + 3, vr.bottom() - 4, QString::number(x, 'g', 4)); }
    for (double y = std::ceil(tl.y() / step) * step; y <= br.y(); y += step) { const QPoint q = mapFromScene(QPointF(0, y)); p->drawText(4, q.y() - 3, QString::number(view_ == Top ? y : -y, 'g', 4)); }
    p->setPen(QColor(60, 60, 60)); QFont fb = f; fb.setBold(true); p->setFont(fb);
    const QString ax = view_ == Top ? "X, м → нос        Y, м ↓ правый борт" : view_ == Side ? "X, м → нос        высота над ватерлинией, м ↑" : "Y, м → правый борт (вид с кормы)        высота над ватерлинией, м ↑";
    p->drawText(vr.left() + 8, vr.top() + 16, ax);
    if (hasImage()) { p->setPen(u_.cal ? QColor(0, 120, 60) : QColor(200, 80, 0)); p->drawText(vr.right() - 190, vr.top() + 16, u_.cal ? "подложка привязана" : "подложка НЕ привязана"); }
    p->restore();
}
