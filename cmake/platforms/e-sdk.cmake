set (CMAKE_SYSTEM_PROCESSOR "riscv" CACHE STRING "")
set (MACHINE                "sifive_u" CACHE STRING "")
set (CROSS_PREFIX           "riscv64-unknown-elf-" CACHE STRING "")
set (PLATFORM_LIB_DEPS      "-lc -lm" CACHE STRING "")
set (CMAKE_C_FLAGS          "-mcmodel=medany -march=rv64imac -mabi=lp64" CACHE STRING "")

include (cross_generic_gcc)
