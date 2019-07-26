# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: xml

add_custom_target(xml.pablosrc 
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/xml.pablo $ENV{HOME}/.cache/parabix
    DEPENDS xml.pablo)

parabix_add_executable(
NAME
    xml
SRC
    post_process.cpp
    test_suite_error.cpp
    xml.cpp
DEPS
    pablo.parse
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.scan
    kernel.streamutils
    kernel.unicode
    toolchain
)

add_dependencies(xml xml.pablosrc)
