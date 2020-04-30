# pinyin counter
# modified ucount.cpp
parabix_add_executable(
NAME
    pycount
SRC
    pycount.cpp
DEPS
    fileselect
    kernel.basis
    kernel.io
    re.cc
    re.parse
)
