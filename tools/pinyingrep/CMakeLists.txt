# This software is licensed to the public under the Open Software License 3.0.

# module: pinyingrep

parabix_add_executable(
NAME
    pinyingrep
SRC
    pinyingrep.cpp
    pinyin_interface.cpp
DEPS 
    fileselect
    grep
    kernel.basis
    kernel.io
    re.cc
    re.parse
)
