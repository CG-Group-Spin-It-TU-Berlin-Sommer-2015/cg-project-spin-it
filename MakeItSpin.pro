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
    view/utility/shader.cpp \
    view/utility/mesh.cpp \
    view/utility/meshreader.cpp \
    view/newglwidget.cpp \
    view/utility/Model.cpp

HEADERS  += view/mainwindow.h \
    view/glwidget.h \
    view/utility/shader.h \
    view/utility/mesh.h \
    view/utility/meshreader.h \
    view/newglwidget.h \
    view/utility/Model.h

FORMS    += view/mainwindow.ui

OTHER_FILES += \
    simple.vsh \
    simple.fsh

RESOURCES += \
    shader.qrc

INCLUDEPATH += $$quote(C:/glm-0.9.6.1/glm/)
