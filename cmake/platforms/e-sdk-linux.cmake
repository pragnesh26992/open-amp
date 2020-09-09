set (CMAKE_SYSTEM_PROCESSOR "riscv")
set (CROSS_PREFIX           "riscv64-unknown-linux-gnu-")
set (CMAKE_C_FLAGS          "-latomic"               CACHE STRING "")
set (MACHINE                "sifive_u_linux" CACHE STRING "")

include (cross_linux_gcc)

# vim: expandtab:ts=2:sw=2:smartindent
