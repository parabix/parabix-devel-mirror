# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: objcache

add_custom_target(recompile-object-cache ALL touch  ${CMAKE_CURRENT_SOURCE_DIR}/object_cache.cpp)

parabix_add_module(objcache
    object_cache.cpp)

target_link_libraries(objcache
    kernel.core)

add_executable(cachejanitord
    object_cache_daemon.cpp)

target_link_libraries(cachejanitord ${Boost_LIBRARIES})
