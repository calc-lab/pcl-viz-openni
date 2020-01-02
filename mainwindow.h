#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "logparser.h"
#include "glviewer.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void slot_timer();
    void slot_btn_start();
    void slot_btn_stop();
private:

    GLViewer *glviewer;
    LogParser logobj;
    QTimer *timer;

    GLuint displaylistbase;
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
