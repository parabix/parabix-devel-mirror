# Copyright (c) 2021 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: icsqlite

#include(ExternalProject)
#ExternalProject_Add(
#  sqlite3build
#  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sqlite
#  CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/sqlite/configure --prefix=${CMAKE_CURRENT_SOURCE_DIR}/sqlite/lib/wow
#  BUILD_COMMAND make
#  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sqlite/build
#)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sqlite/build)

add_custom_target(
  configure-sqlite
  ${SHELL_EXECUTABLE} ../configure
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sqlite/build
)

add_custom_target(
   make-sqlite
   ${MAKE}
   DEPENDS configure-sqlite
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/sqlite/build
)


parabix_add_executable(
NAME
    icsqlite
SRC
    sqlite/build/sqlite3.c
DEPS
    fileselect
    grep
)

add_dependencies(icsqlite make-sqlite)
