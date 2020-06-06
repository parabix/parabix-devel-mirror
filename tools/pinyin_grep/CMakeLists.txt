#module: pinyin_grep

parabix_add_executable(
NAME
    pinyin_grep
SRC
    pinyingrep.cpp
    pinyin.cpp
DEPS
    fileselect
    grep
    kernel.basis
    kernel.io
    re.cc
    re.parse
)
