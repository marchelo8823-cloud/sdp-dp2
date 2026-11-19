// Главное окно редактора данных судна: таблицы книги и три вида чертежа.
#pragma once
#include <QMainWindow>
#include <QPointer>

#include "dp/shipdesign.hpp"

class QTabWidget;
class QTableWidget;
class QLabel;
class QPlainTextEdit;
class QSlider;
class QNetworkAccessManager;
class ViewEdit;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    bool openBook(const QString& path);
    bool saveBook(const QString& path);
    void newShip();
    void setTab(int i);
    ViewEdit* viewEdit(int v) { return views_[v]; }
    void runContour(int view, QRectF px);
    void loadUnderlay(int view, const QString& path);
    void calibrate(int view, QPointF px1, QPointF px2, QPointF m1, QPointF m2);
    void checkBook();
    void applyCalc(int view);
    void recognizeView(int view) { recognize(view); }
    void setHullValue(const QString& label, const QString& v) { setHullVal(label.toUtf8().constData(), v); }
    bool quietDialogs = false;   // режим проверки: окна с вопросами не показываются

protected:
    void closeEvent(QCloseEvent* e) override;

private:
    QWidget* buildView(int v);
    QTableWidget* kvTable(const std::vector<dp::design::KvSpec>& spec, bool reqCol);
    QTableWidget* rowTable(const std::vector<dp::design::ColSpec>& spec);
    void fillKv(QTableWidget* t, const std::vector<dp::design::KvSpec>& spec, const std::map<std::string, std::string>& v);
    std::map<std::string, std::string> readKv(QTableWidget* t, const std::vector<dp::design::KvSpec>& spec) const;
    void fillRows(QTableWidget* t, const std::vector<dp::design::ColSpec>& spec, const std::vector<dp::design::Row>& rows);
    std::vector<dp::design::Row> readRows(QTableWidget* t, const std::vector<dp::design::ColSpec>& spec) const;
    dp::design::ShipDesign gather() const;
    void refreshCalc();
    void refreshMarkers();
    void refreshGuides();
    void refreshScheme();
    void onMarkerMoved(const QString& id, bool thr, double a, double b);
    void askCalibration(int view, QPointF px1, QPointF px2);
    void recognize(int view);
    void claudeSettings();
    void setDirty(bool d = true);
    bool confirmDiscard();
    QString hullVal(const char* label) const;
    void setHullVal(const char* label, const QString& v);
    void say(const QString& s);

    QTabWidget* tabs_ = nullptr;
    QTableWidget *hull_ = nullptr, *thr_ = nullptr, *sen_ = nullptr, *outKv_ = nullptr, *outRows_ = nullptr;
    ViewEdit* views_[3] = {nullptr, nullptr, nullptr};
    QLabel* calc_[3] = {nullptr, nullptr, nullptr};
    QSlider* opac_[3] = {nullptr, nullptr, nullptr};
    QLabel* schemeNote_[3] = {nullptr, nullptr, nullptr};
    QPlainTextEdit* report_ = nullptr;
    QLabel* coord_ = nullptr;
    QString path_;
    bool dirty_ = false, loading_ = false;
    QNetworkAccessManager* net_ = nullptr;
};
