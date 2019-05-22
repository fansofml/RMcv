TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    RMVideoCapture.cpp \
    ArmorFind.cpp \
    ArmorSet.cpp \
    anglesolve.cpp \
    serial.cpp \
    graph.cpp

HEADERS += \
    RMVideoCapture.h \
    ArmorFind.h \
    ArmorSet.h \
    serial.h \
    anglesolve.h \
    graph.h
QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2


LIBS += /usr/local/lib/libopencv_calib3d.so \
/usr/local/lib/libopencv_core.so \
/usr/local/lib/libopencv_highgui.so \
/usr/local/lib/libopencv_imgcodecs.so \
/usr/local/lib/libopencv_imgproc.so \
/usr/local/lib/libopencv_ml.so \
/usr/local/lib/libopencv_objdetect.so \
/usr/local/lib/libopencv_photo.so \
/usr/local/lib/libopencv_shape.so \
/usr/local/lib/libopencv_stitching.so \
/usr/local/lib/libopencv_superres.so \
/usr/local/lib/libopencv_video.so \
/usr/local/lib/libopencv_videoio.so \
/usr/local/lib/libopencv_videostab.so \

LIBS += -lpthread

