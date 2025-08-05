include(${CMAKE_SOURCE_DIR}/cmake/pr_common.cmake)

set(PROJ_NAME pr_openvr)
pr_add_module(${PROJ_NAME} SHARED "addons/virtual_reality/modules/openvr/")

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

find_package(openvr REQUIRED)

pr_add_external_dependency(${PROJ_NAME} openvr LIBRARY PRIVATE)

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
