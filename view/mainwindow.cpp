#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    for (int i = 0; i < this->children().size(); i++) {
        delete this->children().at(i);
    }
    delete ui;
}

/*

*/

void MainWindow::on_loadMeshButton_clicked()
{
    emit
}

void MainWindow::on_hollowButton_clicked()
{
    emit
}
