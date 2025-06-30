set(INSTALL_PATH "addons/virtual_reality/modules/openvr/")
pr_install_create_directory("${INSTALL_PATH}")
pr_install_targets(pr_openvr INSTALL_DIR "${INSTALL_PATH}")

pr_install_binaries(openvr)
