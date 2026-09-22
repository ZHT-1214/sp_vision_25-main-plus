# --------------------------------------------------------------------------------------------
#  FindHikSDK.cmake
#
#  This module finds the HikRobot / Hikvision MVS Camera SDK.
#
#  It defines the following variables:
#    HikSDK_FOUND
#    HikSDK_INCLUDE_DIR
#    HikSDK_LIB
#
#  And the following imported target:
#    hiksdk
# --------------------------------------------------------------------------------------------

# =========================
# 1. SDK 根路径
# =========================
if(WIN32)
  set(HikSDK_Path "$ENV{MVCAM_COMMON_RUNENV}")
else()
  set(HikSDK_Path "$ENV{MVCAM_SDK_PATH}")
endif()

if(NOT HikSDK_Path OR HikSDK_Path STREQUAL "")
  message(STATUS "HikSDK: MVCAM_SDK_PATH is not set")
  set(HikSDK_FOUND FALSE)
  return()
endif()

# =========================
# 2. 查找头文件
# =========================
find_path(
  HikSDK_INCLUDE_DIR
  NAMES
    MvCameraControl.h
    CameraParams.h
    PixelType.h
    MvErrorDefine.h
    MvISPErrorDefine.h
  PATHS
    "${HikSDK_Path}/include"
    "${HikSDK_Path}/Includes"
  NO_DEFAULT_PATH
)

# =========================
# 3. 查找库文件（关键修复点）
# =========================
if(UNIX)
  find_library(
    HikSDK_LIB
    NAMES
      MvCameraControl
      libMvCameraControl.so
    PATHS
      "${HikSDK_Path}/lib"
      "${HikSDK_Path}/lib64"
      "${HikSDK_Path}/lib/arm"
      "${HikSDK_Path}/lib/arm64"
      "${HikSDK_Path}/lib/aarch64"
      "${HikSDK_Path}/lib/x86"
      "${HikSDK_Path}/lib/x64"
      "${HikSDK_Path}/lib/64"
      "${HikSDK_Path}/lib/32"
    NO_DEFAULT_PATH
  )
endif()

# =========================
# 4. Windows（完整但不影响 Linux）
# =========================
if(WIN32)
  find_library(
    HikSDK_LIB
    NAMES MvCameraControl
    PATHS
      "${HikSDK_Path}/Libraries"
      "${HikSDK_Path}/Libraries/win64"
      "${HikSDK_Path}/Libraries/win32"
    NO_DEFAULT_PATH
  )

  find_file(
    HikSDK_DLL
    NAMES MvCameraControl.dll
    PATHS
      "${HikSDK_Path}/Runtime"
      "C:/Program Files (x86)/Common Files/MVS/Runtime"
    NO_DEFAULT_PATH
  )
endif()

# =========================
# 5. 创建导入库目标
# =========================
if(HikSDK_LIB AND HikSDK_INCLUDE_DIR)

  if(NOT TARGET HikSDK::HikSDK)

    add_library(HikSDK::HikSDK SHARED IMPORTED GLOBAL)

    if(WIN32)
      set_target_properties(HikSDK::HikSDK PROPERTIES
        IMPORTED_IMPLIB "${HikSDK_LIB}"
        IMPORTED_LOCATION "${HikSDK_DLL}"
        INTERFACE_INCLUDE_DIRECTORIES "${HikSDK_INCLUDE_DIR}"
      )
    else()
      set_target_properties(HikSDK::HikSDK PROPERTIES
        IMPORTED_LOCATION "${HikSDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${HikSDK_INCLUDE_DIR}"
      )
    endif()

  endif()

endif()
set(HikSDK_LIBS HikSDK::HikSDK)
set(HikSDK_INCLUDE_DIRS ${HikSDK_INCLUDE_DIR})


# =========================
# 6. 标准 find_package 处理
# =========================
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  HikSDK
  REQUIRED_VARS HikSDK_LIB HikSDK_INCLUDE_DIR
)

# =========================
# 7. 调试输出（很重要）
# =========================
if(HikSDK_FOUND)
  message(STATUS "HikSDK found:")
  message(STATUS "  Include dir : ${HikSDK_INCLUDE_DIR}")
  message(STATUS "  Library     : ${HikSDK_LIB}")
else()
  message(STATUS "HikSDK NOT found")
endif()

set(HikSDK_LIBS hiksdk)
set(HikSDK_INCLUDE_DIRS ${HikSDK_INCLUDE_DIR})

mark_as_advanced(HikSDK_LIB HikSDK_INCLUDE_DIR HikSDK_DLL)
