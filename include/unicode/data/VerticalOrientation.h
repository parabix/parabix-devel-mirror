#ifndef VERTICALORIENTATION_H
#define VERTICALORIENTATION_H
/*
 *  Copyright (c) 2018 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 *  This file is generated by UCD_properties.py - manual edits may be lost.
 */

#include "PropertyAliases.h"
#include "PropertyObjects.h"
#include "PropertyValueAliases.h"
#include <unicode/core/unicode_set.h>

namespace UCD {
  namespace VO_ns {
    const unsigned independent_prop_values = 4;
    /** Code Point Ranges for R
    [0000, 00a6], [00a8, 00a8], [00aa, 00ad], [00af, 00b0], [00b2, 00bb],
    [00bf, 00d6], [00d8, 00f6], [00f8, 02e9], [02ec, 10ff], [1200, 1400],
    [1680, 18af], [1900, 2015], [2017, 201f], [2022, 202f], [2032, 203a],
    [203d, 2041], [2043, 2046], [204a, 2050], [2052, 2064], [2066, 20dc],
    [20e1, 20e1], [20e5, 20ff], [2102, 2102], [210a, 210e], [2110, 2112],
    [2115, 2115], [2118, 211d], [2124, 2124], [2126, 2126], [2128, 2128],
    [212a, 212d], [212f, 2134], [2140, 2144], [214b, 214b], [214e, 214e],
    [218a, 218b], [2190, 221d], [221f, 2233], [2236, 22ff], [2308, 230b],
    [2320, 2323], [232c, 237c], [239b, 23bd], [23ce, 23ce], [23d0, 23d0],
    [23dc, 23e1], [2423, 2423], [2500, 259f], [261a, 261f], [2768, 2775],
    [2794, 2b11], [2b30, 2b4f], [2b5a, 2bb7], [2bd2, 2bd2], [2bec, 2bef],
    [2c00, 2e7f], [a4d0, a95f], [a980, abff], [d800, dfff], [fb00, fe0f],
    [fe20, fe2f], [fe49, fe4f], [fe58, fe58], [fe63, fe66], [fe70, ff00],
    [ff0d, ff0d], [ff1c, ff1e], [ff61, ffdf], [ffe8, ffef], [fff9, fffb],
    [fffe, 1097f], [109a0, 1157f], [11600, 119ff], [11ab0, 12fff],
    [13430, 143ff], [14680, 16fdf], [18b00, 1afff], [1b130, 1b16f],
    [1b300, 1cfff], [1d200, 1d2df], [1d380, 1d7ff], [1dab0, 1efff],
    [1f800, 1f8ff], [1fa70, 1ffff], [2fffe, 2ffff], [3fffe, effff],
    [ffffe, fffff], [10fffe, 10ffff]**/


    namespace {
    const static UnicodeSet::run_t __r_Set_runs[] = {
    {Full, 5}, {Mixed, 3}, {Full, 15}, {Mixed, 1}, {Full, 112}, {Empty, 8},
    {Full, 16}, {Mixed, 1}, {Empty, 19}, {Full, 17}, {Mixed, 1}, {Empty, 2},
    {Full, 56}, {Mixed, 4}, {Full, 2}, {Mixed, 5}, {Empty, 1}, {Mixed, 1},
    {Full, 3}, {Mixed, 2}, {Full, 6}, {Mixed, 2}, {Full, 1}, {Mixed, 5},
    {Empty, 1}, {Mixed, 1}, {Empty, 6}, {Full, 5}, {Empty, 3}, {Mixed, 1},
    {Empty, 10}, {Mixed, 2}, {Full, 27}, {Mixed, 3}, {Full, 2}, {Mixed, 3},
    {Full, 20}, {Empty, 946}, {Mixed, 1}, {Full, 36}, {Empty, 1},
    {Full, 20}, {Empty, 352}, {Full, 64}, {Empty, 216}, {Full, 24},
    {Mixed, 4}, {Full, 4}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Full, 3},
    {Mixed, 1}, {Full, 76}, {Empty, 1}, {Full, 95}, {Empty, 4}, {Full, 32},
    {Empty, 5}, {Mixed, 1}, {Full, 170}, {Empty, 33}, {Mixed, 1},
    {Full, 126}, {Empty, 20}, {Full, 331}, {Empty, 217}, {Full, 296},
    {Empty, 9}, {Mixed, 1}, {Full, 1}, {Mixed, 1}, {Empty, 12}, {Full, 232},
    {Empty, 16}, {Full, 7}, {Empty, 5}, {Full, 36}, {Empty, 21}, {Mixed, 1},
    {Full, 170}, {Empty, 64}, {Full, 8}, {Empty, 11}, {Mixed, 1},
    {Full, 44}, {Empty, 2047}, {Mixed, 1}, {Empty, 2047}, {Mixed, 1},
    {Full, 22528}, {Empty, 2047}, {Mixed, 1}, {Empty, 2047}, {Mixed, 1}};
    const static UnicodeSet::bitquad_t  __r_Set_quads[] = {
    0x8ffdbd7f, 0xff7fffff, 0xff7fffff, 0xfffff3ff, 0x00000001, 0x0000ffff,
    0xffbfffff, 0xe7fcfffc, 0xfffdfc7b, 0xffffffdf, 0x1fffffff, 0xffffffe2,
    0x3f277c04, 0x001fbd50, 0x0000481f, 0xffff0c00, 0xbfffffff, 0xffcfffff,
    0x00000f00, 0xfffff00f, 0x1fffffff, 0xf8000000, 0x3fffffff, 0xf0014000,
    0x00000003, 0x00000008, 0xfc000000, 0x003fff00, 0xfff00000, 0x0003ffff,
    0xffff0000, 0xfc00ffff, 0x00ffffff, 0x00040000, 0x0000f000, 0xffff0000,
    0x0000ffff, 0x0000ffff, 0x0100fe00, 0xffff0078, 0x70002001, 0xfffffffe,
    0xce00ff00, 0xffff0000, 0xffff0000, 0xffff0000, 0x0000ffff, 0xffff0000,
    0xffff0000, 0xc0000000, 0xc0000000, 0xc0000000, 0xc0000000};
    }

    const static UnicodeSet r_Set{const_cast<UnicodeSet::run_t *>(__r_Set_runs), 95, 0, const_cast<UnicodeSet::bitquad_t *>(__r_Set_quads), 53, 0};

    /** Code Point Ranges for U
    [00a7, 00a7], [00a9, 00a9], [00ae, 00ae], [00b1, 00b1], [00bc, 00be],
    [00d7, 00d7], [00f7, 00f7], [02ea, 02eb], [1100, 11ff], [1401, 167f],
    [18b0, 18ff], [2016, 2016], [2020, 2021], [2030, 2031], [203b, 203c],
    [2042, 2042], [2047, 2049], [2051, 2051], [2065, 2065], [20dd, 20e0],
    [20e2, 20e4], [2100, 2101], [2103, 2109], [210f, 210f], [2113, 2114],
    [2116, 2117], [211e, 2123], [2125, 2125], [2127, 2127], [2129, 2129],
    [212e, 212e], [2135, 213f], [2145, 214a], [214c, 214d], [214f, 2189],
    [218c, 218f], [221e, 221e], [2234, 2235], [2300, 2307], [230c, 231f],
    [2324, 2328], [232b, 232b], [237d, 239a], [23be, 23cd], [23cf, 23cf],
    [23d1, 23db], [23e2, 2422], [2424, 24ff], [25a0, 2619], [2620, 2767],
    [2776, 2793], [2b12, 2b2f], [2b50, 2b59], [2bb8, 2bd1], [2bd3, 2beb],
    [2bf0, 2bff], [2e80, 3000], [3003, 3007], [3012, 3013], [3020, 302f],
    [3031, 3040], [3042, 3042], [3044, 3044], [3046, 3046], [3048, 3048],
    [304a, 3062], [3064, 3082], [3084, 3084], [3086, 3086], [3088, 308d],
    [308f, 3094], [3097, 309a], [309d, 309f], [30a2, 30a2], [30a4, 30a4],
    [30a6, 30a6], [30a8, 30a8], [30aa, 30c2], [30c4, 30e2], [30e4, 30e4],
    [30e6, 30e6], [30e8, 30ed], [30ef, 30f4], [30f7, 30fb], [30fd, 3126],
    [3128, 31ef], [3200, 32ff], [3358, 337a], [3380, a4cf], [a960, a97f],
    [ac00, d7ff], [e000, faff], [fe10, fe1f], [fe30, fe48], [fe53, fe57],
    [fe5f, fe62], [fe67, fe6f], [ff02, ff07], [ff0a, ff0b], [ff0f, ff19],
    [ff20, ff3a], [ff3c, ff3c], [ff3e, ff3e], [ff40, ff5a], [ffe0, ffe2],
    [ffe4, ffe7], [fff0, fff8], [fffc, fffd], [10980, 1099f],
    [11580, 115ff], [11a00, 11aaf], [13000, 1342f], [14400, 1467f],
    [16fe0, 18aff], [1b000, 1b12f], [1b170, 1b2ff], [1d000, 1d1ff],
    [1d2e0, 1d37f], [1d800, 1daaf], [1f000, 1f1ff], [1f202, 1f7ff],
    [1f900, 1fa6f], [20000, 2fffd], [30000, 3fffd], [f0000, ffffd],
    [100000, 10fffd]**/


    namespace {
    const static UnicodeSet::run_t __u_Set_runs[] = {
    {Empty, 5}, {Mixed, 3}, {Empty, 15}, {Mixed, 1}, {Empty, 112},
    {Full, 8}, {Empty, 16}, {Mixed, 1}, {Full, 19}, {Empty, 17}, {Mixed, 1},
    {Full, 2}, {Empty, 56}, {Mixed, 4}, {Empty, 2}, {Mixed, 5}, {Full, 1},
    {Mixed, 1}, {Empty, 3}, {Mixed, 2}, {Empty, 6}, {Mixed, 2}, {Empty, 1},
    {Mixed, 5}, {Full, 1}, {Mixed, 1}, {Full, 6}, {Empty, 5}, {Full, 3},
    {Mixed, 1}, {Full, 10}, {Mixed, 2}, {Empty, 27}, {Mixed, 3}, {Empty, 2},
    {Mixed, 3}, {Empty, 20}, {Full, 12}, {Mixed, 8}, {Full, 1}, {Mixed, 1},
    {Full, 5}, {Mixed, 1}, {Full, 8}, {Empty, 2}, {Mixed, 2}, {Full, 906},
    {Mixed, 1}, {Empty, 36}, {Full, 1}, {Empty, 20}, {Full, 352},
    {Empty, 64}, {Full, 216}, {Empty, 24}, {Mixed, 4}, {Empty, 4},
    {Mixed, 3}, {Empty, 4}, {Mixed, 1}, {Empty, 76}, {Full, 1}, {Empty, 95},
    {Full, 4}, {Empty, 32}, {Full, 5}, {Mixed, 1}, {Empty, 170}, {Full, 33},
    {Mixed, 1}, {Empty, 126}, {Full, 20}, {Empty, 331}, {Full, 217},
    {Empty, 296}, {Full, 9}, {Mixed, 1}, {Empty, 1}, {Mixed, 1}, {Full, 12},
    {Empty, 232}, {Full, 16}, {Empty, 7}, {Full, 5}, {Empty, 36},
    {Full, 21}, {Mixed, 1}, {Empty, 170}, {Full, 16}, {Mixed, 1},
    {Full, 47}, {Empty, 8}, {Full, 11}, {Mixed, 1}, {Empty, 44},
    {Full, 2047}, {Mixed, 1}, {Full, 2047}, {Mixed, 1}, {Empty, 22528},
    {Full, 2047}, {Mixed, 1}, {Full, 2047}, {Mixed, 1}};
    const static UnicodeSet::bitquad_t  __u_Set_quads[] = {
    0x70024280, 0x00800000, 0x00800000, 0x00000c00, 0xfffffffe, 0xffff0000,
    0x00400000, 0x18030003, 0x00020384, 0x00000020, 0xe0000000, 0x0000001d,
    0xc0d883fb, 0xffe042af, 0xffffb7e0, 0x0000f3ff, 0x40000000, 0x00300000,
    0xfffff0ff, 0x000009f0, 0xe0000000, 0x07ffffff, 0xc0000000, 0x0ffebfff,
    0xfffffffc, 0xfffffff7, 0x03ffffff, 0xffc000ff, 0x000fffff, 0xfffc0000,
    0x0000ffff, 0x03ff0000, 0xff000000, 0xfffbffff, 0xffff0fff, 0x000c00f9,
    0xfffeffff, 0xfffffd55, 0xfffffff7, 0xe79fbf57, 0xfffffd54, 0xfffffff7,
    0xef9fbf57, 0xffffff7f, 0x0000ffff, 0xff000000, 0x07ffffff, 0x0000ffff,
    0xffff0000, 0xffff0000, 0x80f801ff, 0x0000ff87, 0x03ff8cfc, 0x57ffffff,
    0x07ffffff, 0x31ff00f7, 0x0000ffff, 0x0000ffff, 0x0000ffff, 0xffff0000,
    0x0000ffff, 0xfffffffc, 0x0000ffff, 0x3fffffff, 0x3fffffff, 0x3fffffff,
    0x3fffffff};
    }

    const static UnicodeSet u_Set{const_cast<UnicodeSet::run_t *>(__u_Set_runs), 104, 0, const_cast<UnicodeSet::bitquad_t *>(__u_Set_quads), 67, 0};

    /** Code Point Ranges for Tr
    [2329, 232a], [3008, 3011], [3014, 301f], [3030, 3030], [30a0, 30a0],
    [30fc, 30fc], [fe59, fe5e], [ff08, ff09], [ff1a, ff1b], [ff3b, ff3b],
    [ff3d, ff3d], [ff3f, ff3f], [ff5b, ff60], [ffe3, ffe3]**/


    namespace {
    const static UnicodeSet::run_t __tr_Set_runs[] = {
    {Empty, 281}, {Mixed, 1}, {Empty, 102}, {Mixed, 2}, {Empty, 3},
    {Mixed, 1}, {Empty, 1}, {Mixed, 1}, {Empty, 1642}, {Mixed, 1},
    {Empty, 5}, {Mixed, 4}, {Empty, 3}, {Mixed, 1}, {Empty, 32768}};
    const static UnicodeSet::bitquad_t  __tr_Set_quads[] = {
    0x00000600, 0xfff3ff00, 0x00010000, 0x00000001, 0x10000000, 0x7e000000,
    0x0c000300, 0xa8000000, 0xf8000000, 0x00000001, 0x00000008};
    }

    const static UnicodeSet tr_Set{const_cast<UnicodeSet::run_t *>(__tr_Set_runs), 15, 0, const_cast<UnicodeSet::bitquad_t *>(__tr_Set_quads), 11, 0};

    /** Code Point Ranges for Tu
    [3001, 3002], [3041, 3041], [3043, 3043], [3045, 3045], [3047, 3047],
    [3049, 3049], [3063, 3063], [3083, 3083], [3085, 3085], [3087, 3087],
    [308e, 308e], [3095, 3096], [309b, 309c], [30a1, 30a1], [30a3, 30a3],
    [30a5, 30a5], [30a7, 30a7], [30a9, 30a9], [30c3, 30c3], [30e3, 30e3],
    [30e5, 30e5], [30e7, 30e7], [30ee, 30ee], [30f5, 30f6], [3127, 3127],
    [31f0, 31ff], [3300, 3357], [337b, 337f], [fe50, fe52], [ff01, ff01],
    [ff0c, ff0c], [ff0e, ff0e], [ff1f, ff1f], [1f200, 1f201]**/


    namespace {
    const static UnicodeSet::run_t __tu_Set_runs[] = {
    {Empty, 384}, {Mixed, 1}, {Empty, 1}, {Mixed, 6}, {Empty, 1},
    {Mixed, 1}, {Empty, 5}, {Mixed, 1}, {Empty, 8}, {Full, 2}, {Mixed, 2},
    {Empty, 1622}, {Mixed, 1}, {Empty, 5}, {Mixed, 1}, {Empty, 1943},
    {Mixed, 1}, {Empty, 30831}};
    const static UnicodeSet::bitquad_t  __tu_Set_quads[] = {
    0x00000006, 0x000002aa, 0x00000008, 0x186040a8, 0x000002aa, 0x00000008,
    0x006040a8, 0x00000080, 0xffff0000, 0x00ffffff, 0xf8000000, 0x00070000,
    0x80005002, 0x00000003};
    }

    const static UnicodeSet tu_Set{const_cast<UnicodeSet::run_t *>(__tu_Set_runs), 18, 0, const_cast<UnicodeSet::bitquad_t *>(__tu_Set_quads), 14, 0};

    static EnumeratedPropertyObject property_object
        {vo,
        VO_ns::independent_prop_values,
        std::move(VO_ns::enum_names),
        std::move(VO_ns::value_names),
        std::move(VO_ns::aliases_only_map),{
        &r_Set, &u_Set, &tr_Set, &tu_Set
        }};
    }
}

#endif