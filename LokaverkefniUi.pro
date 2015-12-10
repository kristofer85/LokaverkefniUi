#-------------------------------------------------
#
# Project created by QtCreator 2015-12-10T19:58:05
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LokaverkefniUi
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    stereocalibrate.cpp \
    stereoscopicimage.cpp \
    convert.cpp \
    pclfilters.cpp \
    visualizer.cpp \
    test.cpp \
    reprojectimageto3d.cpp \
    utils.cpp \
    depthmap.cpp \
    Rectify.cpp \
    histogram.cpp \
    depthmap.cpp

HEADERS  += mainwindow.h \
    stereocalibrate.h \
    stereoscopicimage.h \
    defines.h \
    convert.h \
    pclfilters.h \
    visualizer.h \
    test.h \
    reprojectimageto3d.h \
    utils.h \
    depthmap.h \
    Rectify.h \
    histogram.h

FORMS    += mainwindow.ui


#INCLUDEPATH += C:/msys64/mingw32/include/
INCLUDEPATH += C:/msys64/home/Notandi/opencv/build2/install/include/
INCLUDEPATH += C:/msys64/home/Notandi/vtk/build/install/include/vtk-6.2
INCLUDEPATH += C:/msys64/home/Notandi/cloudstuff/pl-build/install/include/pcl-1.8
INCLUDEPATH += C:/msys64/mingw32/include/eigen3
INCLUDEPATH += C:/msys64/home/Notandi/cloudstuff/flann-1.8.4-src/build/install/include
INCLUDEPATH += C:/msys64/mingw32/include/json
INCLUDEPATH += C:/msys64/mingw32/include/glib-2.0/
INCLUDEPATH += C:/msys64/mingw32/include/
INCLUDEPATH += C:/msys64/mingw32/lib/glib-2.0/include/
win32: LIBS += -LC:/msys64/mingw32/i686-w64-mingw32/lib \
                -lkernel32 \
                -lpsapi \
                -luser32
win32: LIBS += -LC:/msys64/home/Notandi/vtk/build/install/lib \
               -LC:/msys64/home/Notandi/cloudstuff/pl-build/install/lib \
               -LC:/msys64/home/Notandi/opencv/build2/install/x86/mingw/lib \
               -LC:/msys64/bin \
                -lopencv_calib3d \
                -lopencv_core300 \
                -lopencv_features2d300 \
                -lopencv_flann300 \
                -lopencv_highgui300 \
                -lopencv_imgproc300 \
                -lopencv_hal300 \
                -lopencv_ccalib300 \
                -lopencv_imgcodecs300 \
                -lopencv_objdetect300 \
                #-lopencv_photo300 \
                #-lopencv_stitching300 \
                #-lopencv_superres300 \
                #-lopencv_ts300 \
                #-lopencv_video300 \
                #-lopencv_videostab300 \
                -lopencv_ml300 \
                #-lopencv_objdetect300 \
                #-lopencv_photo300 \
                #-lopencv_shape300 \
                #-lopencv_stitching300 \
                #-lopencv_superres300 \
                #-lopencv_videoio300 \
                -lopencv_ximgproc300 \
                #-lopencv_bgsegm300 \
                -lopencv_xfeatures2d300 \
                -lopencv_stereo300 \
                -lpcl_common \
                -lpcl_visualization \
                -lpcl_io \
                -lpcl_io_ply \
                -lpcl_filters \
                -lpcl_features \
                -lpcl_2d \
                -lpcl_keypoints \
                -lpcl_ml \
                -lpcl_octree \
                -lpcl_outofcore \
                -lpcl_people \
                -lpcl_recognition \
                -lpcl_sample_consensus \
                -lpcl_segmentation \
                -lpcl_stereo \
                -lpcl_tracking \
                -lpcl_search \
                -lpcl_kdtree \
                -lpcl_surface \
                -lvtkalglib-6.2 \
                -lvtkChartsCore-6.2 \
                -lvtkCommonColor-6.2 \
                -lvtkCommonComputationalGeometry-6.2 \
                -lvtkCommonCore-6.2 \
                -lvtkCommonDataModel-6.2 \
                -lvtkCommonExecutionModel-6.2 \
                -lvtkCommonMath-6.2 \
                -lvtkCommonMisc-6.2 \
                -lvtkCommonSystem-6.2 \
                -lvtkCommonTransforms-6.2 \
                -lvtkGUISupportQt-6.2 \
                -lvtkRenderingAnnotation-6.2 \
                -lvtkRenderingContext2D-6.2 \
                -lvtkRenderingCore-6.2 \
                -lvtkRenderingFreeType-6.2 \
                -lvtkRenderingFreeTypeOpenGL-6.2 \
                -lvtkRenderingGL2PS-6.2 \
                #-lvtkRenderingHybridOpenGL-6.2 \
                -lvtkRenderingImage-6.2 \
                -lvtkRenderingLOD-6.2 \
                -lvtkRenderingLabel-6.2 \
                -lvtkRenderingOpenGL-6.2 \
                -lvtkRenderingVolume-6.2 \
                -lboost_system-mt \
                -lboost_thread-mt \
                -lboost_filesystem-mt\
                -ljsoncpp \
                #-ljson-glib-1.0-0\
                -lexiv2 \
                -llensfun
