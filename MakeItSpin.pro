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
    view/utility/kdtree.cpp \
    view/utility/meshwriter.cpp \
    view/utility/simplemeshmerger.cpp \
    view/utility/basicoctree.cpp \
    view/utility/extendedoctree.cpp

HEADERS  += view/mainwindow.h \
    model/model.h \
    view/glwidget.h \
    view/utility/shader.h \
    view/utility/mesh.h \
    view/utility/meshreader.h \
    view/utility/kdtree.h \
    view/utility/meshwriter.h \
    view/utility/simplemeshmerger.h \
    view/utility/basicoctree.h \
    view/utility/extendedoctree.h

FORMS    += view/mainwindow.ui

OTHER_FILES += \
    simple.vsh \
    simple.fsh

RESOURCES += \
    shader.qrc


#unix:!macx: LIBS += -Lusr/lib/ -lpcl_octree

#INCLUDEPATH += /usr/include/eigen3

#INCLUDEPATH += /usr/include/pcl-1.7
#DEPENDPATH += /usr/include/pcl-1.7
