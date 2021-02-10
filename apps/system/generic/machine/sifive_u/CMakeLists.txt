+collect (APP_COMMON_SOURCES helper.c)
+
+set (_linker_script "${CMAKE_CURRENT_SOURCE_DIR}/linker_remote.ld")
+
+set_property (GLOBAL PROPERTY APP_LINKER_OPT "-Wl,-build-id=none -specs=${CMAKE_CURRENT_SOURCE_DIR}/Xilinx.spec -T\"${_linker_script}\"")
+
+collect(PROJECT_LIB_DEPS c)
+collect(PROJECT_LIB_DEPS m)

