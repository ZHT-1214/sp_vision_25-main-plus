# FindTensorRT.cmake -- Locate NVIDIA TensorRT

include(FindPackageHandleStandardArgs)

# TensorRT root
if (DEFINED TensorRT_ROOT)
  list(APPEND _TensorRT_SEARCH_PATHS
    ${TensorRT_ROOT}
    "$ENV{TensorRT_ROOT}"
  )
endif()
list(APPEND _TensorRT_SEARCH_PATHS /usr /usr/local)

# Header
find_path(TensorRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES include
)

# Core library
find_library(TensorRT_LIBRARY
  NAMES nvinfer 
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES lib lib64 lib/x64
)

find_package_handle_standard_args(TensorRT
  REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_LIBRARY
)

if (TensorRT_FOUND)
  set(TensorRT_INCLUDE_DIRS ${TensorRT_INCLUDE_DIR})
  set(TensorRT_LIBRARIES ${TensorRT_LIBRARY})

  # Optional components
  foreach(_comp IN ITEMS nvinfer_plugin nvonnxparser nvparsers)
    find_library(TensorRT_${_comp}_LIBRARY
      NAMES ${_comp}
      PATHS ${_TensorRT_SEARCH_PATHS}
      PATH_SUFFIXES lib lib64 lib/x64
    )
    if (TensorRT_${_comp}_LIBRARY)
      list(APPEND TensorRT_LIBRARIES ${TensorRT_${_comp}_LIBRARY})
    endif()
  endforeach()

  # Core target
  add_library(TensorRT::TensorRT UNKNOWN IMPORTED)
  set_target_properties(TensorRT::TensorRT PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${TensorRT_LIBRARY}"
  )

  # Component targets
  foreach(_comp IN ITEMS nvinfer_plugin nvonnxparser nvparsers)
    if (TensorRT_${_comp}_LIBRARY)
      add_library(TensorRT::${_comp} UNKNOWN IMPORTED)
      set_target_properties(TensorRT::${_comp} PROPERTIES
        IMPORTED_LOCATION "${TensorRT_${_comp}_LIBRARY}"
      )
    endif()
  endforeach()

  message(STATUS "Found TensorRT at ${TensorRT_INCLUDE_DIR}")
endif()
