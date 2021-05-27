# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.scan

parabix_add_module(
NAME
    kernel.scan
SRC
    base.cpp
    index_generator.cpp
    line_number_generator.cpp
    line_span_generator.cpp
    reader.cpp
    scanmatchgen.cpp
DEPS
    kernel.pipeline
    kernel.streamutils
)
