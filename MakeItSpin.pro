#-------------------------------------------------
#
# Project created by QtCreator 2015-05-05T23:17:34
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MakeItSpin
TEMPLATE = app


SOURCES += main.cpp \
    view/mainwindow.cpp \
    view/glwidget.cpp \
    view/utility/shader.cpp \
    view/utility/meshreader.cpp \
    view/utility/meshwriter.cpp \
    optimization/octree/extendedoctree.cpp \
    optimization/octree/basicoctree.cpp \
    optimization/betaOptimization/betaoptimization.cpp \
    mesh/meshOperations/meshbooleanfunctions.cpp \
    mesh/meshOperations/simplemeshmerger.cpp \
    mesh/mesh.cpp

HEADERS  += view/mainwindow.h \
    view/glwidget.h \
    view/utility/shader.h \
    view/utility/meshreader.h \
    view/utility/meshwriter.h \
    optimization/octree/extendedoctree.h \
    optimization/octree/basicoctree.h \
    optimization/betaOptimization/betaoptimization.h \
    mesh/meshOperations/meshbooleanfunctions.h \
    mesh/meshOperations/simplemeshmerger.h \
    mesh/mesh.h

FORMS    += view/mainwindow.ui

OTHER_FILES += \
    simple.vsh \
    simple.fsh

RESOURCES += \
    shader.qrc


unix:!macx: LIBS += -L$$PWD/../cork/lib/ -lcork

INCLUDEPATH += $$PWD/../cork/include
DEPENDPATH += $$PWD/../cork/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../cork/lib/libcork.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lgmp

INCLUDEPATH += $$PWD/../../../../usr/include/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/include/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libgmp.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lnlopt
INCLUDEPATH += $$PWD/../../../../usr/local/include/
DEPENDPATH += $$PWD/../../../../usr/local/include/
unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/local/lib/libnlopt.a
