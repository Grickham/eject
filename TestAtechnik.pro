#-------------------------------------------------
#
# Project created by QtCreator 2017-06-19T16:06:37
#
#-------------------------------------------------

QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TestAtechnik
TEMPLATE = app
CONFIG  += C++11

SOURCES += main.cpp\
        at_vista.cpp \
    at_controlador.cpp \
    at_modelo.cpp \
    at_aviso.cpp

HEADERS  += at_vista.h \
    at_controlador.h \
    at_modelo.h \
    estado_programa.h \
    at_aviso.h

FORMS    += at_vista.ui \
    at_aviso.ui

unix:!macx: LIBS += -L$$PWD/../build-Libreria_ATGUI-Desktop_Qt_5_5_1_GCC_64bit-Debug/ -lLibreria_ATGUI

INCLUDEPATH += $$PWD/../build-Libreria_ATGUI-Desktop_Qt_5_5_1_GCC_64bit-Debug
DEPENDPATH += $$PWD/../build-Libreria_ATGUI-Desktop_Qt_5_5_1_GCC_64bit-Debug

unix:!macx: LIBS += -lpthread -lLibAutomatischeMitsu

RESOURCES += \
    assets.qrc
