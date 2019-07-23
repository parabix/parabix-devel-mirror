# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: toolchain

parabix_add_module(toolchain
    pablo_toolchain.cpp
    toolchain.cpp)

# Embed whole LLVMSupport static library in the toolchain shared library to
# allow LLVM's command line library to work properly.
if(APPLE)
    # Use Darwin only '-all_load' flag on macOS.
    set(ALL_LOAD_BEGIN "")
    set(ALL_LOAD_END "-all_load")
else()
    # Use GNU --whole-archive option on Linux.
    set(ALL_LOAD_BEGIN "-Wl,--whole-archive")
    set(ALL_LOAD_END "-Wl,--no-whole-archive")
endif(APPLE)

# Remove LLVMSupport from the list of LLVM libraries as to no link with it twice.
list(REMOVE_ITEM REQ_LLVM_LIBRARIES LLVMSupport)

target_link_libraries(toolchain
    ${ALL_LOAD_BEGIN}
    LLVMSupport
    ${ALL_LOAD_END}
    ${REQ_LLVM_LIBRARIES})
