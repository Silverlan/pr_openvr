include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME pr_openvr)
pr_add_library(${PROJ_NAME} SHARED)

pr_add_dependency(${PROJ_NAME} ishared TARGET PUBLIC)
pr_add_dependency(${PROJ_NAME} iclient TARGET PUBLIC)

pr_add_headers(${PROJ_NAME} "include/")
pr_add_sources(${PROJ_NAME} "src/")

#TODO: Point this to proper arch lib
if(WIN32)
    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "X86")
        set(OPENVR_ARCH "win32")
    elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "AMD64")
        set(OPENVR_ARCH "win64")
    endif()
elseif(UNIX AND NOT APPLE)
    # We need util_archive on Linux to determine the Steam installation path (See OpenVrInitializer::Initialize)
    pr_add_dependency(${PROJ_NAME} util_archive TARGET PRIVATE)

    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i386")
        set(OPENVR_ARCH "linux32")
    elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
        set(OPENVR_ARCH "linux64")
    elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
        set(OPENVR_ARCH "linuxarm64")
    endif()
#elseif(APPLE)
#    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i386" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
#        set(OPENVR_ARCH "osx32") #fat/universal bin
#    endif()
#elseif(ANDROID)
#    if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
#        set(OPENVR_ARCH "androidarm64" )
#    endif()
endif()

if("${OPENVR_ARCH}" STREQUAL "")
    MESSAGE("Invalid architecture! Got: System: ${CMAKE_SYSTEM_NAME} Arch: ${CMAKE_SYSTEM_PROCESSOR}")
    return()
endif()

target_include_directories(${PROJ_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/headers/")
if(WIN32)
    target_link_libraries(${PROJ_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/lib/win64/openvr_api.lib")
else()
    target_link_libraries(${PROJ_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr/lib/linux64/libopenvr_api.so")
endif()

if(UNIX)
    set_property(TARGET ${PROJ_NAME}
    APPEND PROPERTY INSTALL_RPATH
        "$ORIGIN:$ORIGIN/../../../../lib"
    )
    set_property(TARGET ${PROJ_NAME}
    APPEND PROPERTY INSTALL_RPATH
        "$ORIGIN/addons/virtual_reality/modules/openvr"
    )
endif()

pr_finalize(${PROJ_NAME} FOLDER "${IDE_FOLDER}")
