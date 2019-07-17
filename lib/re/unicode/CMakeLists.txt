# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.unicode

parabix_add_module(re.unicode
    casing.cpp
    decomposition.cpp
    equivalence.cpp
    grapheme_clusters.cpp
    re_name_resolve.cpp
    resolve_properties.cpp)

target_link_libraries(re.unicode
    re.compile
    unicode.data)
