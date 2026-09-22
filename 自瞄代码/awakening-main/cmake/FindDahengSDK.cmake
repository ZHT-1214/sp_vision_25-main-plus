# --------------------------------------------------------------------------------------------
# FindDahengSDK.cmake
# --------------------------------------------------------------------------------------------

set(DahengSDK_Path "$ENV{DAHENG_SDK_PATH}")
if(NOT DahengSDK_Path OR DahengSDK_Path STREQUAL "")
  unset(DahengSDK_Path)
endif()

find_path(
  DahengSDK_INCLUDE_DIR
  NAMES
    GxIAPI.h
    DxImageProc.h
  PATHS
    /usr/include
    /usr/local/include
    /opt
    ${DahengSDK_Path}
    ${DahengSDK_Path}/include
    ${DahengSDK_Path}/inc
    ${DahengSDK_Path}/Includes
  PATH_SUFFIXES
    galaxy_camera
    Galaxy_camera
    GalaxyCamera
    include
    inc
)

find_library(
  DahengSDK_LIB
  NAMES
    gxiapi
    libgxiapi.so
  PATHS
    /lib
    /usr/lib
    /usr/local/lib
    /lib/x86_64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    ${DahengSDK_Path}
    ${DahengSDK_Path}/lib
    ${DahengSDK_Path}/lib64
)

if(DahengSDK_LIB AND DahengSDK_INCLUDE_DIR)
  if(NOT TARGET DahengSDK::DahengSDK)
    add_library(DahengSDK::DahengSDK SHARED IMPORTED GLOBAL)
    set_target_properties(DahengSDK::DahengSDK PROPERTIES
      IMPORTED_LOCATION "${DahengSDK_LIB}"
      INTERFACE_INCLUDE_DIRECTORIES "${DahengSDK_INCLUDE_DIR}"
    )
  endif()
endif()

set(DahengSDK_LIBS DahengSDK::DahengSDK)
set(DahengSDK_INCLUDE_DIRS ${DahengSDK_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  DahengSDK
  REQUIRED_VARS DahengSDK_LIB DahengSDK_INCLUDE_DIR
)

if(DahengSDK_FOUND)
  message(STATUS "DahengSDK found:")
  message(STATUS "  include: ${DahengSDK_INCLUDE_DIR}")
  message(STATUS "  lib    : ${DahengSDK_LIB}")
else()
  message(STATUS "DahengSDK NOT found")
endif()

mark_as_advanced(DahengSDK_LIB DahengSDK_INCLUDE_DIR)
