INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_SOAPYMIMO soapymimo)

FIND_PATH(
    SOAPYMIMO_INCLUDE_DIRS
    NAMES soapymimo/api.h
    HINTS $ENV{SOAPYMIMO_DIR}/include
        ${PC_SOAPYMIMO_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    SOAPYMIMO_LIBRARIES
    NAMES gnuradio-soapymimo
    HINTS $ENV{SOAPYMIMO_DIR}/lib
        ${PC_SOAPYMIMO_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SOAPYMIMO DEFAULT_MSG SOAPYMIMO_LIBRARIES SOAPYMIMO_INCLUDE_DIRS)
MARK_AS_ADVANCED(SOAPYMIMO_LIBRARIES SOAPYMIMO_INCLUDE_DIRS)

