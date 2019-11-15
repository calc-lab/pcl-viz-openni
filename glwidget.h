#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QPaintEvent>

#include "define.h"

enum DrawingType{
    POINTS,
    LINES,
    PLANES
};

struct DrawingElem{
    std::vector<point3fi> pts;
    std::vector<color3b> colors;
    int pointsize;
    DrawingType type;
};

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

    void setXRot(int angle);

    void setYRot(int angle);

    void setZRot(int angle);

    void setScale(int value);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    QSize minimumSizeHint() const;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);

public:
    void setPointCloudFrame(std::vector<DrawingElem> elems);

private:
    void draw();
    void drawAxis(double length);
    void drawPointCloud();

    int xRot;
    int yRot;
    int zRot;

    int zEye;

    int xEye, yEye;

    QPoint lastPos;

    QList<int> selections;

    std::vector <DrawingElem> drawingelements;
};

#endif // GLWIDGET_H
