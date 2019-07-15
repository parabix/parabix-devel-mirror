# This software is licensed to the public under the Open Software License 3.0.

# module: xml

add_custom_target(xml.pablosrc 
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/xml.pablo $ENV{HOME}/.cache/parabix
    DEPENDS xml.pablo)

add_executable(xml
    post_process.cpp
    test_suite_error.cpp
    xml.cpp)

add_dependencies(xml xml.pablosrc)

target_link_libraries(xml
    pablo
    pablo.parse
    kernel.core
    kernel.pipeline
    kernel.util
    toolchain)
