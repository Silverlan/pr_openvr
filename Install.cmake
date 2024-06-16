set(INSTALL_PATH "addons/virtual_reality/modules/openvr/")
pr_install_create_directory("${INSTALL_PATH}")
pr_install_targets(pr_openvr INSTALL_DIR "${INSTALL_PATH}")

if(WIN32)
    pr_install_files(
        "${CMAKE_CURRENT_LIST_DIR}/third_party/openvr/bin/win64/openvr_api.dll"
        INSTALL_DIR "${INSTALL_PATH}"
    )
else()
    pr_install_files(
        "${CMAKE_CURRENT_LIST_DIR}/third_party/openvr/bin/linux64/libopenvr_api.so"
        INSTALL_DIR "${INSTALL_PATH}"
    )
endif()
