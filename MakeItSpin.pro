#-------------------------------------------------
#
# Project created by QtCreator 2015-05-05T23:17:34
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MakeItSpin
TEMPLATE = app


SOURCES += main.cpp\
    view/mainwindow.cpp \
    view/glwidget.cpp \
    model/objreader.cpp \
    view/utility/shader.cpp \
    view/utility/mesh.cpp

HEADERS  += view/mainwindow.h \
    view/glwidget.h \
    view/objreader.h \
    model/objreader.h \
    view/utility/shader.h \
    view/utility/mesh.h

FORMS    += view/mainwindow.ui

OTHER_FILES += \
    simple.vsh \
    simple.fsh

RESOURCES += \
    shader.qrc

