
#include "view/mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setFixedWidth(1000);
    w.setFixedHeight(600);

    w.move( 0, 0 );
    w.show();

    return a.exec();
}
