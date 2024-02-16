cmake_minimum_required(VERSION 3.12)

include("./CMakeShared.txt")

set(PROJ_NAME pr_openvr)

project(${PROJ_NAME} CXX)


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




set(DEPENDENCY_OPENVR_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/third_party/openvr/headers CACHE PATH "Path to include directory.")
search_lib_recursive(DEPENDENCY_OPENVR_LIBRARY third_party/openvr/lib/${OPENVR_ARCH}/ openvr_api DYNAMIC ${CMAKE_CURRENT_LIST_DIR})

# set(PRECOMPILED_HEADER "stdafx")

link_external_library(iclient)
link_external_library(openvr)
link_external_library(glfw)
link_external_library(iglfw)
link_external_library(prosper)
link_external_library(anvil)
link_external_library(vulkan)
link_external_library(glslang)
link_external_library(wgui)
add_external_library(luajit)
link_external_library(luasystem)
link_external_library(luabind)
link_external_library(mathutil)
link_external_library(sharedutils)
link_external_library(vfilesystem)
link_external_library(datasystem)
link_external_library(shared)
link_external_library(ishared)
link_external_library(lua)

add_include_dir(glm)
add_include_dir(anvil_build)
add_include_dir(iglfw)
add_include_dir(ishared)
add_include_dir(mathutil)
add_include_dir(shared)
add_include_dir(client)
add_include_dir(vfilesystem)
add_include_dir(datasystem)
add_include_dir(sharedutils)
add_include_dir(alsoundsystem)
add_include_dir(materialsystem)
add_include_dir(cmaterialsystem)
add_include_dir(util_udm)

add_def(ANVIL_VULKAN_CPP)
add_def(USE_VULKAN)
add_def(ALSYS_LIBRARY_TYPE)

set(DEPENDENCY_OPENVR_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/third_party/openvr/headers CACHE PATH "" FORCE)
if(WIN32)
	set(DEPENDENCY_OPENVR_LIBRARY ${CMAKE_CURRENT_LIST_DIR}/third_party/openvr/lib/win64/openvr_api.lib CACHE PATH "" FORCE)
else()
	set(DEPENDENCY_OPENVR_LIBRARY ${CMAKE_CURRENT_LIST_DIR}/third_party/openvr/lib/linux64/libopenvr_api.so CACHE PATH "" FORCE)
endif()
# link_generic_module_libraries()

add_subdirectory(third_party/openvr)
if(WIN32)
	set_target_properties(openvr_api64 PROPERTIES FOLDER third_party)
else()
	set_target_properties(openvr_api PROPERTIES FOLDER third_party)
endif()



register_pragma_module(${PROJ_NAME})

add_dependencies(${PROJ_NAME} Anvil)
