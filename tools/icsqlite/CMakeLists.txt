# Copyright (c) 2021 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: icsqlite

parabix_add_executable(
NAME
    icsqlite
SRC
    sqlite/sqlite3.c
DEPS
    fileselect
    grep
)
