
/*
 *  Copyright (c) 2021 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 *  This file is generated by UCD_properties.py - manual edits may be lost.
 *  DerivedBinaryProperties
 */

#include <unicode/core/unicode_set.h>
#include <unicode/data/PropertyAliases.h>
#include <unicode/data/PropertyObjects.h>
#include <unicode/data/PropertyValueAliases.h>

namespace UCD {
    namespace BIDI_M_ns {
        /** Code Point Ranges for Bidi_M
        [0028, 0029], [003c, 003c], [003e, 003e], [005b, 005b],
        [005d, 005d], [007b, 007b], [007d, 007d], [00ab, 00ab],
        [00bb, 00bb], [0f3a, 0f3d], [169b, 169c], [2039, 203a],
        [2045, 2046], [207d, 207e], [208d, 208e], [2140, 2140],
        [2201, 2204], [2208, 220d], [2211, 2211], [2215, 2216],
        [221a, 221d], [221f, 2222], [2224, 2224], [2226, 2226],
        [222b, 2233], [2239, 2239], [223b, 224c], [2252, 2255],
        [225f, 2260], [2262, 2262], [2264, 226b], [226e, 228c],
        [228f, 2292], [2298, 2298], [22a2, 22a3], [22a6, 22b8],
        [22be, 22bf], [22c9, 22cd], [22d0, 22d1], [22d6, 22ed],
        [22f0, 22ff], [2308, 230b], [2320, 2321], [2329, 232a],
        [2768, 2775], [27c0, 27c0], [27c3, 27c6], [27c8, 27c9],
        [27cb, 27cd], [27d3, 27d6], [27dc, 27de], [27e2, 27ef],
        [2983, 2998], [299b, 29a0], [29a2, 29af], [29b8, 29b8],
        [29c0, 29c5], [29c9, 29c9], [29ce, 29d2], [29d4, 29d5],
        [29d8, 29dc], [29e1, 29e1], [29e3, 29e5], [29e8, 29e9],
        [29f4, 29f9], [29fc, 29fd], [2a0a, 2a1c], [2a1e, 2a21],
        [2a24, 2a24], [2a26, 2a26], [2a29, 2a29], [2a2b, 2a2e],
        [2a34, 2a35], [2a3c, 2a3e], [2a57, 2a58], [2a64, 2a65],
        [2a6a, 2a6d], [2a6f, 2a70], [2a73, 2a74], [2a79, 2aa3],
        [2aa6, 2aad], [2aaf, 2ad6], [2adc, 2adc], [2ade, 2ade],
        [2ae2, 2ae6], [2aec, 2aee], [2af3, 2af3], [2af7, 2afb],
        [2afd, 2afd], [2bfe, 2bfe], [2e02, 2e05], [2e09, 2e0a],
        [2e0c, 2e0d], [2e1c, 2e1d], [2e20, 2e29], [3008, 3011],
        [3014, 301b], [fe59, fe5e], [fe64, fe65], [ff08, ff09],
        [ff1c, ff1c], [ff1e, ff1e], [ff3b, ff3b], [ff3d, ff3d],
        [ff5b, ff5b], [ff5d, ff5d], [ff5f, ff60], [ff62, ff63],
        [1d6db, 1d6db], [1d715, 1d715], [1d74f, 1d74f], [1d789, 1d789],
        [1d7c3, 1d7c3]**/


        namespace {
        const static UnicodeSet::run_t __codepoint_set_runs[] = {
        {Empty, 1}, {Mixed, 3}, {Empty, 1}, {Mixed, 1}, {Empty, 115},
        {Mixed, 1}, {Empty, 58}, {Mixed, 1}, {Empty, 76}, {Mixed, 4},
        {Empty, 5}, {Mixed, 1}, {Empty, 5}, {Mixed, 10}, {Empty, 33},
        {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 12}, {Mixed, 8},
        {Full, 1}, {Mixed, 3}, {Empty, 7}, {Mixed, 1}, {Empty, 16},
        {Mixed, 2}, {Empty, 14}, {Mixed, 1}, {Empty, 1649}, {Mixed, 2},
        {Empty, 4}, {Mixed, 4}, {Empty, 1722}, {Mixed, 1}, {Empty, 1},
        {Mixed, 1}, {Empty, 1}, {Mixed, 1}, {Empty, 1}, {Mixed, 1},
        {Empty, 1}, {Mixed, 1}, {Empty, 31041}};
        const static UnicodeSet::bitquad_t  __codepoint_set_quads[] = {
        0x50000300, 0x28000000, 0x28000000, 0x08000800, 0x3c000000,
        0x18000000, 0x06000000, 0x00000060, 0x60000000, 0x00006000,
        0x00000001, 0xbc623f1e, 0xfa0ff857, 0x803c1fff, 0xffffcff5,
        0x01079fff, 0xc1ffffcc, 0xffc33e00, 0xffff3fff, 0x00000f00,
        0x00000603, 0x003fff00, 0x70783b79, 0x0000fffc, 0xf9fffff8,
        0x0100fffd, 0x1f37c23f, 0x33f0033a, 0xdffffc00, 0x70307a53,
        0x01800000, 0xfe19bc30, 0xffffbfcf, 0x507fffff, 0x2f88707c,
        0x40000000, 0x3000363c, 0x000003ff, 0x0ff3ff00, 0x7e000000,
        0x00000030, 0x50000300, 0x28000000, 0xa8000000, 0x0000000d,
        0x08000000, 0x00200000, 0x00008000, 0x00000200, 0x00000008};
        }

        const static UnicodeSet codepoint_set{const_cast<UnicodeSet::run_t *>(__codepoint_set_runs), 43, 0, const_cast<UnicodeSet::bitquad_t *>(__codepoint_set_quads), 50, 0};

        static BinaryPropertyObject property_object{Bidi_M, std::move(codepoint_set)};
    }
PropertyObject * get_BIDI_M_PropertyObject() {  return & BIDI_M_ns::property_object; }
}
