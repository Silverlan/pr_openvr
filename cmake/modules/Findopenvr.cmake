set(PCK "openvr")

if (${PCK}_FOUND)
  return()
endif()

find_path(${PCK}_INCLUDE_DIR
  NAMES openvr.h
  HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/headers
)

find_library(${PCK}_LIBRARY
  NAMES openvr_api
  HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/lib/linux64
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/lib/win64
)

set(REQ_VARS ${PCK}_LIBRARY ${PCK}_INCLUDE_DIR)
if(WIN32)
  find_file(
    ${PCK}_RUNTIME
    NAMES openvr_api.dll
    HINTS
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/bin/win64
  )
  set(REQ_VARS ${REQ_VARS} ${PCK}_RUNTIME)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(${PCK}
  REQUIRED_VARS ${REQ_VARS}
)

if(${PCK}_FOUND)
  set(${PCK}_LIBRARIES   ${${PCK}_LIBRARY})
  set(${PCK}_INCLUDE_DIRS ${${PCK}_INCLUDE_DIR})
endif()
