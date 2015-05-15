#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QVBoxLayout>
#include <QPushButton>
#include <QGridLayout>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{

    // initiate QWidget and set width and height
    QWidget *window = new QWidget();
    window->setMinimumWidth(500);
    window->setMinimumHeight(400);

    this->setCentralWidget( window );
    QGridLayout *lay = new QGridLayout( window);
    QWidget *buttonWidget = new QWidget();

    // initiate NewGLWidget and set width and height
    w = new NewGLWidget();
    w->setFixedWidth(800);
    w->setFixedHeight(500);

    lay->addWidget(w,0,0);
    lay->addWidget(buttonWidget,0,1);

    window->setLayout(lay);

    QGridLayout *buttonLay = new QGridLayout( buttonWidget);

    buttonWidget->setLayout(buttonLay);

    QPushButton *button;

    int index = 0;

    // set open button
    button = new QPushButton(buttonWidget);
    button->setText(tr("Open"));
    button->setMinimumWidth(100);
    button->setMaximumWidth(100);
    button->setMaximumHeight(25);
    buttonLay->addWidget(button,index++,0);
    connect(button, SIGNAL(released()), this, SLOT(open()));

    // set save button
    button = new QPushButton(buttonWidget);
    button->setText(tr("Save"));
    button->setMinimumWidth(100);
    button->setMaximumWidth(100);
    button->setMaximumHeight(25);
    buttonLay->addWidget(button,index++,0);
    connect(button, SIGNAL(released()), this, SLOT(save()));

    // set button for set test setting
    button = new QPushButton(buttonWidget);
    button->setText(tr("Set Cube"));
    button->setMinimumWidth(100);
    button->setMaximumWidth(100);
    button->setMaximumHeight(25);
    buttonLay->addWidget(button,index++,0);
    connect(button, SIGNAL(released()), this, SLOT(setTestSetting1()));

    // set button for set test setting
    button = new QPushButton(buttonWidget);
    button->setText(tr("Set Cuboid"));
    button->setMinimumWidth(100);
    button->setMaximumWidth(100);
    button->setMaximumHeight(25);
    buttonLay->addWidget(button,index++,0);
    connect(button, SIGNAL(released()), this, SLOT(setTestSetting2()));

    // set button for set test setting
    button = new QPushButton(buttonWidget);
    button->setText(tr("Set DummyBoy"));
    button->setMinimumWidth(100);
    button->setMaximumWidth(100);
    button->setMaximumHeight(25);
    buttonLay->addWidget(button,index++,0);
    connect(button, SIGNAL(released()), this, SLOT(setTestSetting3()));

    buttonLay->setAlignment(Qt::AlignTop);

    setWindowTitle(tr("Spin It"));

}

MainWindow::~MainWindow()
{
}

/*
 * callback for set test setting event
 */
void MainWindow::setTestSetting1()
{

}

/*
 * callback for set test setting event
 */
void MainWindow::setTestSetting2()
{

}

/*
 * callback for set test setting event
 */
void MainWindow::setTestSetting3()
{

}

/*
 * callback for open button event
 */
void MainWindow::open()
{

}

/*
 * callback for save button event
 */
void MainWindow::save()
{

}

/*
 * callback for key press event
 */
void MainWindow::keyPressEvent(QKeyEvent* e)
{

};

/*
 * callback for key release event
 */
void MainWindow::keyReleaseEvent(QKeyEvent *e)
{

}

/*
 * callback for wheel event
 */
void MainWindow::wheelEvent(QWheelEvent* event)
{

}
