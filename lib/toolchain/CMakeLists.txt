# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: toolchain

add_custom_target(recompile-object-cache ALL touch  ${CMAKE_CURRENT_SOURCE_DIR}/object_cache.cpp)

add_library(toolchain
    cpudriver.cpp
    driver.cpp
    # NVPTXDriver.cpp
    object_cache.cpp
    toolchain.cpp)

parabix_set_library_name(toolchain)

target_link_libraries(toolchain kernel.core)

add_executable(cachejanitord
    object_cache_daemon.cpp)

target_link_libraries(cachejanitord ${Boost_LIBRARIES})
