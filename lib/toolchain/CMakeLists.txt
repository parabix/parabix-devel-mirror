# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: toolchain

# IMPORTANT:
#
# If an explicit link with LLVM is needed, link with this library indead.
#
# i.e.,
#   DO:
#       target_link_libraries(<mylib> toolchain)
#
#   DON'T:
#       target_link_libraries(<mylib> ${REQ_LLVM_LIBRARIES})

parabix_add_module(toolchain
    pablo_toolchain.cpp
    toolchain.cpp)

if(APPLE)
    # The Darwin linker on macOS does not support the --whole-archive flag.
    # Instead it has -all_load which does something similar. On macOS we need
    # to load all of the static LLVM libraries into this shared library in order
    # for symbol resolution of global varaibles in LLVM's Command Line library
    # to work as expected.
    #
    # We also need to declare this link PRIVATE or else it won't work. I don't
    # know why as it seems very counter intuitive, but it doesn't work any other
    # way.
    target_link_libraries(toolchain PRIVATE -all_load ${REQ_LLVM_LIBRARIES})
else()
    # On Linux we use --whole-archive to embed the symbols from LLVMSupport into
    # this shared library so that, when other libraries need to locate the global
    # variables used in LLVM's Command Line library they will find them here and
    # not import new isntances from LLVM (hence the need to link with this library
    # and not directly to LLVM).
    #
    # We can't embed symbols from all LLVM libraries as the GNU linker fails with
    # multiple definition errors (this doesn't happend with the Darwin linker).
    target_link_libraries(toolchain
        -Wl,--whole-archive LLVMSupport -Wl,--no-whole-archive
        ${REQ_LLVM_LIBRARIES})
endif(APPLE)
