TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.c \
    ../libs/ur5.c \
    ../libs/ur5lib.c \
    ../libs/ur_kin.c \
    ../libs/helper.c \
    ../libs/tcphelper.c \
    ../libs/interrupt_utils.c \
    ../libs/startup_utils.c \
    ../libs/base_utils.c
    ../libs/startup_utils.c

HEADERS += \
    ../libs/robotinterface.h \
    ../libs/ur5lib.h \
    ../libs/ur_kin.h \
    ../libs/helper.h \
    ../libs/tcphelper.h \
    ../libs/interrupt_utils.h \
    ../libs/startup_utils.h \
    ../libs/base_utils.h
    ../libs/startup_utils.h


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../ur5test/libs/release/ -lur5
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../ur5test/libs/debug/ -lur5
else:unix: LIBS += -L$$PWD/../ur5test/libs/ -lur5

INCLUDEPATH += $$PWD/../ur5test/libs
DEPENDPATH += $$PWD/../ur5test/libs

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../ur5test/libs/release/libur5.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../ur5test/libs/debug/libur5.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../ur5test/libs/release/ur5.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../ur5test/libs/debug/ur5.lib
else:unix: PRE_TARGETDEPS += $$PWD/../ur5test/libs/libur5.a

unix|win32: LIBS += -lpthread

INCLUDEPATH += $$PWD/../libs
DEPENDPATH += $$PWD/../libs
OTHER_FILES += \
    scons \
    SConstruct \
    SConscript
