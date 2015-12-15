#-------------------------------------------------
#
# Project created by QtCreator 2015-12-10T19:58:05
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LokaverkefniUi
TEMPLATE = app
#QMAKE_CXXFLAGS += -std=c++11


SOURCES += main.cpp\
        mainwindow.cpp \
    stereocalibrate.cpp \
    convert.cpp \
    visualizer.cpp \
    utils.cpp \
    Rectify.cpp \
    histogram.cpp \
    depthmap.cpp

HEADERS  += mainwindow.h \
    stereocalibrate.h \
    defines.h \
    convert.h \
    visualizer.h \
    utils.h \
    depthmap.h \
    Rectify.h \
    histogram.h

FORMS    += mainwindow.ui


INCLUDEPATH += C:/msys64/mingw32/include/
INCLUDEPATH += C:/msys64/home/kristinn/opencv/build/install/include/
INCLUDEPATH += C:/msys64/home/kristinn/vtk/build2/install/include/vtk-6.3
INCLUDEPATH += C:/msys64/home/kristinn/cloudstuff/pl-build/install/include/pcl-1.8
INCLUDEPATH += C:/msys64/mingw32/include/eigen3
INCLUDEPATH += C:/msys64/home/kristinn/cloudstuff/flann-1.8.4-src/build/install/include
INCLUDEPATH += C:/msys64/mingw32/include/json
INCLUDEPATH += C:/msys64/mingw32/include/glib-2.0/
INCLUDEPATH += C:/msys64/mingw32/lib/glib-2.0/include/
win32: LIBS += -LC:/msys64/home/kristinn/vtk/build2/install/lib \
	       -LC:/msys64/home/kristinn/cloudstuff/pl-build/install/lib \
	       -LC:/msys64/home/kristinn/opencv/build/install/x86/mingw/lib \
		-lopencv_calib3d \
		-lopencv_core300d \
		-lopencv_features2d300d \
		-lopencv_flann300d \
		-lopencv_highgui300d \
		-lopencv_imgproc300d \
		-lopencv_hal300d \
		-lopencv_ccalib300d \
		-lopencv_imgcodecs300d \
		-lopencv_objdetect300d \
		-lopencv_photo300d \
		-lopencv_stitching300d \
		-lopencv_superres300d \
		-lopencv_ts300d \
		-lopencv_video300d \
		-lopencv_videostab300d \
		-lopencv_ml300d \
		-lopencv_objdetect300d \
		-lopencv_photo300d \
		-lopencv_shape300d \
		-lopencv_stitching300d \
		-lopencv_superres300d \
		-lopencv_videoio300d \
		-lopencv_ximgproc300d \
		-lopencv_bgsegm300d \
		-lopencv_xfeatures2d300d \
		-lopencv_stereo300d \
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
		-lvtkalglib-6.3 \
		-lvtkChartsCore-6.3 \
		-lvtkCommon-6.3 \
		-lvtkDICOMParser-6.3 \
		-lvtkDomainsChemistry-6.3 \
		-lvtkexoIIc-6.3 \
		-lvtkexpat-6.3 \
		-lvtkFilters-6.3 \
		-lvtkFiltersFlowPaths-6.3 \
		-lvtkFiltersHybrid-6.3 \
		-lvtkfreetype-6.3 \
		-lvtkftgl-6.3 \
		-lvtkGeovisCore-6.3 \
		-lvtkgl2ps-6.3 \
		-lvtkGUISupportQt-6.3 \
		-lvtkGUISupportQtOpenGL-6.3 \
		-lvtkGUISupportQtSQL-6.3 \
		-lvtkGUISupportQtWebkit-6.3 \
		-lvtkImaging-6.3 \
		-lvtkImagingHybrid-6.3 \
		-lvtkInfovisCore-6.3 \
		-lvtkInfovisLayout-6.3 \
		-lvtkInteraction-6.3 \
		-lvtkIO-6.3 \
		-lvtkIOExport-6.3 \
		-lvtkIOImport-6.3 \
		-lvtkIOInfovis-6.3 \
		-lvtkIOMINC-6.3 \
		-lvtkjpeg-6.3 \
		-lvtkjsoncpp-6.3 \
		-lvtklibxml2-6.3 \
		-lvtkmetaio-6.3 \
		-lvtkNetCDF_cxx-6.3 \
		-lvtkNetCDF-6.3 \
		-lvtkoggtheora-6.3 \
		-lvtkOpenGL-6.3 \
		-lvtkParallel-6.3 \
		-lvtkpng-6.3 \
		-lvtkproj4-6.3 \
		-lvtkRendering-6.3 \
		-lvtkRenderingQt-6.3 \
		-lvtksqlite-6.3 \
		-lvtksys-6.3 \
		-lvtktiff-6.3 \
		-lvtkverdict-6.3 \
		-lvtkViews-6.3 \
		-lvtkViewsInfovis-6.3 \
		-lvtkViewsQt-6.3 \
		-lvtkzlib-6.3 \
		-lboost_system-mt \
		-lboost_thread-mt \
		-lboost_filesystem-mt \
		-ljsoncpp \
		#-ljson-glib-1.0-0\
		-lexiv2 \
		-llensfun
