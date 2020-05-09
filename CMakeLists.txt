cmake_minimum_required(VERSION 3.5)

project(design LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})

# pcl
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

#Boost
#set(BOOST_ROOT "/usr/local/boost") #boost库
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#VTK
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

#resources
file(GLOB_RECURSE QRC_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.qrc)
set(QRC_SOURCE_FILES imag.qrc)
qt5_add_resources(QRC_FILES ${QRC_SOURCE_FILES})


if(ANDROID)
  add_library(design SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
else()
  add_executable(design
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    paramdialog.cpp
    paramdialog.h
    paramdialog.ui
    ../../../clion/pclDemo/learning/designLib/tunnelTool.cpp
    ../../../clion/pclDemo/learning/designLib/tunnelTool.h
    ../../../clion/pclDemo/learning/pcdFile/calibratingTool.h
    ../../../clion/pclDemo/learning/pcdFile/groundHelper.h
    ../../../clion/pclDemo/learning/pcdFile/pclDBSCAN.h
    ../../../clion/pclDemo/learning/pcdFile/rockfaceHelper.h
    ../../../clion/pclDemo/learning/pcdFile/steelArchHelper.h
    ${QRC_FILES}
  )
endif()

target_link_libraries(design PRIVATE Qt5::Widgets
           ${VTK_LIBRARIES}
           ${Boost_LIBRARIES}
           ${PCL_LIBRARIES})
