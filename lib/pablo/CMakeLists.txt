# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: pablo

parabix_add_module(
NAME
    pablo
SRC
    arithmetic.cpp
    branch.cpp
    builder.cpp
    carry_manager.cpp
    codegenstate.cpp
    codemotionpass.cpp
    compressed_carry_manager.cpp
    distributivepass.cpp
    flattenif.cpp
    # pablo_automultiplexing.cpp # TODO: use source variable
    passes.cpp
    pablo_compiler.cpp
    pablo_kernel.cpp
    pablo_simplifier.cpp
    pabloAST.cpp
    pabloverifier.cpp
    printer_pablos.cpp
    schedulingprepass.cpp
    symbol_generator.cpp
DEPS
    kernel.core
)
