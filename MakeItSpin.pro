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
    view/utility/simplemeshmerger.cpp \
    view/utility/basicoctree.cpp \
    view/utility/extendedoctree.cpp \
    view/utility/extendedmeshmerger.cpp

HEADERS  += view/mainwindow.h \
    model/model.h \
    view/glwidget.h \
    view/utility/shader.h \
    view/utility/mesh.h \
    view/utility/meshreader.h \
    view/utility/meshwriter.h \
    view/utility/simplemeshmerger.h \
    view/utility/basicoctree.h \
    view/utility/extendedoctree.h \
    view/utility/extendedmeshmerger.h

FORMS    += view/mainwindow.ui

OTHER_FILES += \
    simple.vsh \
    simple.fsh

RESOURCES += \
    shader.qrc

#unix:!macx: LIBS += -Lusr/lib/ -lpcl_octree
#INCLUDEPATH += /usr/include/pcl-1.7
#DEPENDPATH += /usr/include/pcl-1.7

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

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../cork/lib/release/ -lcork
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../cork/lib/debug/ -lcork
#else:unix: LIBS += -L$$PWD/../cork/lib/ -lcork

#INCLUDEPATH += $$PWD/../cork/include
#DEPENDPATH += $$PWD/../cork/include

#win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../cork/lib/release/libcork.a
#else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../cork/lib/debug/libcork.a
#else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../cork/lib/release/cork.lib
#else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../cork/lib/debug/cork.lib
#else:unix: PRE_TARGETDEPS += $$PWD/../cork/lib/libcork.a
