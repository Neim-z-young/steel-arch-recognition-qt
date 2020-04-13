QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
     ../../../clion/pclDemo/learning/designLib/tunnelTool.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/calibratingTool.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/calibration.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/clusterByDBSCAN.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/example_spin_images.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/groundHelper.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/groundRemove.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/pclDBSCAN.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/rockfaceExtraction.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/rockfaceHelper.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/show.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/steelArchExtraction.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/steelArchHelper.cpp \
#    ../../../clion/pclDemo/learning/pcdFile/voxelization.cpp \
    main.cpp \
    mainwindow.cpp \
    paramdialog.cpp

HEADERS += \
    ../../../clion/pclDemo/learning/designLib/tunnelTool.h \
    ../../../clion/pclDemo/learning/pcdFile/calibratingTool.h \
    ../../../clion/pclDemo/learning/pcdFile/groundHelper.h \
    ../../../clion/pclDemo/learning/pcdFile/pclDBSCAN.h \
    ../../../clion/pclDemo/learning/pcdFile/rockfaceHelper.h \
    ../../../clion/pclDemo/learning/pcdFile/steelArchHelper.h \
    mainwindow.h \
    paramdialog.h

unix{
    #eigen3
    INCLUDEPATH += /usr/include/eigen3

    #vtk
    INCLUDEPATH += /usr/local/include/vtk-8.2
    LIBS += /usr/local/lib/libvtk*.so

    #boost
    INCLUDEPATH += /usr/include/boost
    #LIBS += /usr/lib/libboost_*.so

    #pcl
    INCLUDEPATH += /usr/local/include/pcl-1.9
    LIBS += /usr/local/lib/libpcl_*.so
}


FORMS += \
    mainwindow.ui \
    paramdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
