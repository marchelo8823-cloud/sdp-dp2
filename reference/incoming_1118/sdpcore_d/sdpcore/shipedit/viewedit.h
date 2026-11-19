// Редактор одного вида судна (сверху, сбоку, спереди): сетка в метрах, подложка-картинка с привязкой,
// простые фигуры (прямоугольник, многоугольник), метки движителей и датчиков на виде сверху.
#pragma once
#include <QGraphicsView>
#include <QImage>
#include <QPointer>
#include <vector>

#include "dp/shipdesign.hpp"

class QGraphicsPixmapItem;
class QGraphicsPolygonItem;
class QGraphicsRectItem;
class QGraphicsPathItem;

struct Marker { QString id; bool thruster = true; double a = 0, b = 0; QString kind; };

class ViewEdit : public QGraphicsView {
    Q_OBJECT
public:
    enum Tool { Select, Rect, Poly, Calib, Contour };
    explicit ViewEdit(int view, QWidget* parent = nullptr);

    int view() const { return view_; }
    void setTool(Tool t);
    Tool tool() const { return tool_; }

    void setShapes(const std::vector<dp::design::Shape>& s);
    std::vector<dp::design::Shape> shapes() const;
    void addShapePx(const std::vector<QPointF>& px, const QString& name);      // фигура в пикселях подложки
    void deleteSelected();

    bool loadImage(const QString& path, QString& err);
    void setUnderlay(const dp::design::Underlay& u, const QString& baseDir);
    dp::design::Underlay underlay() const { return u_; }
    void setCalibration(QPointF px1, QPointF px2, QPointF m1, QPointF m2);
    void setOpacityValue(double o);
    bool hasImage() const { return !img_.isNull(); }
    bool calibrated() const { return u_.cal && hasImage(); }
    const QImage& image() const { return img_; }
    QString imagePath() const { return imgPath_; }

    void setGuides(double draft, double length, double beam) { draft_ = draft; len_ = length; beam_ = beam; viewport()->update(); }
    void setMarkers(const QVector<Marker>& m);
    // схема по данным корпуса: пунктир, не выбирается и в расчёт не входит
    void setScheme(const dp::design::Scheme& s);
    void setSchemeVisible(bool on);
    std::vector<dp::design::Shape> schemeShapes() const;
    void addShapes(const std::vector<dp::design::Shape>& s);
    void fit();

signals:
    void changed();
    void markerMoved(const QString& id, bool thruster, double a, double b);
    void cursorAt(double a, double b);
    void calibPicked(QPointF px1, QPointF px2);
    void contourPicked(QRectF pxRect);
    void message(const QString& m);

protected:
    void drawBackground(QPainter* p, const QRectF& r) override;
    void drawForeground(QPainter* p, const QRectF& r) override;
    void wheelEvent(QWheelEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;

private:
    void layoutLabels();   // разнести подписи меток, чтобы не налезали друг на друга (в пикселях экрана)
    QPointF toScene(double a, double b) const { return {a, view_ == dp::design::Top ? b : -b}; }
    void fromScene(QPointF s, double& a, double& b) const { a = s.x(); b = view_ == dp::design::Top ? s.y() : -s.y(); }
    QGraphicsPolygonItem* makeShape(const QPolygonF& poly, const QString& name);
    void updateHandles();
    void applyUnderlayTransform();
    QPointF sceneToPx(QPointF s) const;
    QGraphicsPolygonItem* selectedShape() const;
    int vertexAt(QGraphicsPolygonItem* it, QPoint vp) const;
    void finishPoly();
    void cancelDraw();
    void styleShape(QGraphicsPolygonItem* it, bool sel);

    int view_; Tool tool_ = Select;
    QGraphicsScene* sc_;
    QGraphicsPixmapItem* pix_ = nullptr; QImage img_; QString imgPath_;
    dp::design::Underlay u_;
    std::vector<QGraphicsPolygonItem*> items_;
    QVector<Marker> markers_; std::vector<QGraphicsItem*> markerItems_;
    double draft_ = 0, len_ = 0, beam_ = 0;
    // текущие действия мышью
    enum Drag { None, Pan, MoveShape, MoveVertex, DrawRect, PickRect, MoveMarker } drag_ = None;
    QPoint lastVp_; QPointF startSc_; int vtx_ = -1; int marker_ = -1;
    QGraphicsRectItem* rubber_ = nullptr;
    QPolygonF poly_; QGraphicsPathItem* polyPreview_ = nullptr;
    std::vector<QPointF> calibPx_;
    QGraphicsPathItem* handles_ = nullptr;
    std::vector<QGraphicsItem*> schemeItems_; std::vector<dp::design::Shape> scheme_; bool schemeOn_ = true;
};
