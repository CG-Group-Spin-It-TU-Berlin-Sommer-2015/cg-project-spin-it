#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include "newglwidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void setTestSetting1();
    void setTestSetting2();
    void setTestSetting3();
    void open();
    void save();

private:
    Ui::MainWindow *ui;
    NewGLWidget *w;

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent *e);
    void wheelEvent(QWheelEvent* event);

};

#endif // MAINWINDOW_H
