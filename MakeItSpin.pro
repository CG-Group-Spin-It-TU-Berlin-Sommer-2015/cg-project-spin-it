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
    model/model.cpp \
    view/mainwindow.cpp \
    view/glwidget.cpp \
    view/utility/shader.cpp \
    view/utility/mesh.cpp \
    view/utility/meshreader.cpp \
    view/utility/meshwriter.cpp \
    optimization/octree/extendedoctree.cpp \
    optimization/octree/basicoctree.cpp \
    optimization/meshOperations/simplemeshmerger.cpp \
    optimization/meshOperations/meshbooleanfunctions.cpp \
    optimization/betaOptimization/betaoptimization.cpp \
    optimization/betaOptimization/betaoptimization2.cpp

HEADERS  += view/mainwindow.h \
    model/model.h \
    view/glwidget.h \
    view/utility/shader.h \
    view/utility/mesh.h \
    view/utility/meshreader.h \
    view/utility/meshwriter.h \
    optimization/octree/extendedoctree.h \
    optimization/octree/basicoctree.h \
    optimization/meshOperations/simplemeshmerger.h \
    optimization/meshOperations/meshbooleanfunctions.h \
    optimization/betaOptimization/betaoptimization.h \
    optimization/betaOptimization/betaoptimization2.h

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

INCLUDEPATH += $$PWD/../eigen
DEPENDPATH += $$PWD/../eigen

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lgmp

INCLUDEPATH += $$PWD/../../../../usr/include/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/include/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libgmp.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lnlopt
INCLUDEPATH += $$PWD/../../../../usr/local/include/
DEPENDPATH += $$PWD/../../../../usr/local/include/
unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/local/lib/libnlopt.a
