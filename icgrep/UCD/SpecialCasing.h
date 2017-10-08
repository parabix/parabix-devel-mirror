#ifndef SPECIALCASING_H
#define SPECIALCASING_H
/*
 *  Copyright (c) 2017 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 *  This file is generated by UCD_properties.py - manual edits may be lost.
 */

#include "PropertyAliases.h"
#include "PropertyObjects.h"
#include "PropertyValueAliases.h"
#include "UnicodeData.h"
#include "unicode_set.h"

namespace UCD {
    namespace LC_ns {
        /** Code Point Ranges for lc (possibly overriding values from SLC)
        [00df, 00df], [0130, 0130], [0149, 0149], [01f0, 01f0],
        [0390, 0390], [03b0, 03b0], [0587, 0587], [1e96, 1e9a],
        [1f50, 1f50], [1f52, 1f52], [1f54, 1f54], [1f56, 1f56],
        [1f80, 1faf], [1fb2, 1fb4], [1fb6, 1fb7], [1fbc, 1fbc],
        [1fc2, 1fc4], [1fc6, 1fc7], [1fcc, 1fcc], [1fd2, 1fd3],
        [1fd6, 1fd7], [1fe2, 1fe4], [1fe6, 1fe7], [1ff2, 1ff4],
        [1ff6, 1ff7], [1ffc, 1ffc], [fb00, fb06], [fb13, fb17]**/

        const UnicodeSet explicitly_defined_set
                    {{{Empty, 6}, {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 4},
              {Mixed, 1}, {Empty, 12}, {Mixed, 2}, {Empty, 14}, {Mixed, 1},
              {Empty, 199}, {Mixed, 1}, {Empty, 5}, {Mixed, 1}, {Empty, 1},
              {Full, 1}, {Mixed, 3}, {Empty, 1752}, {Mixed, 1},
              {Empty, 32807}},
             {0x80000000, 0x00010000, 0x00000200, 0x00010000, 0x00010000,
              0x00010000, 0x00000080, 0x07c00000, 0x00550000, 0x10dcffff,
              0x00cc10dc, 0x10dc00dc, 0x00f8007f}};

        const unsigned buffer_length = 406;
        const static char __attribute__ ((aligned (32))) string_buffer[512] = u8R"__(ß
i̇
ŉ
ǰ
ΐ
ΰ
և
ẖ
ẗ
ẘ
ẙ
ẚ
ὐ
ὒ
ὔ
ὖ
ᾀ
ᾁ
ᾂ
ᾃ
ᾄ
ᾅ
ᾆ
ᾇ
ᾀ
ᾁ
ᾂ
ᾃ
ᾄ
ᾅ
ᾆ
ᾇ
ᾐ
ᾑ
ᾒ
ᾓ
ᾔ
ᾕ
ᾖ
ᾗ
ᾐ
ᾑ
ᾒ
ᾓ
ᾔ
ᾕ
ᾖ
ᾗ
ᾠ
ᾡ
ᾢ
ᾣ
ᾤ
ᾥ
ᾦ
ᾧ
ᾠ
ᾡ
ᾢ
ᾣ
ᾤ
ᾥ
ᾦ
ᾧ
ᾲ
ᾳ
ᾴ
ᾶ
ᾷ
ᾳ
ῂ
ῃ
ῄ
ῆ
ῇ
ῃ
ῒ
ΐ
ῖ
ῗ
ῢ
ΰ
ῤ
ῦ
ῧ
ῲ
ῳ
ῴ
ῶ
ῷ
ῳ
ﬀ
ﬁ
ﬂ
ﬃ
ﬄ
ﬅ
ﬆ
ﬓ
ﬔ
ﬕ
ﬖ
ﬗ
)__";

        const static std::vector<codepoint_t> defined_cps = {
        0x00df, 0x0130, 0x0149, 0x01f0, 0x0390, 0x03b0, 0x0587, 0x1e96,
        0x1e97, 0x1e98, 0x1e99, 0x1e9a, 0x1f50, 0x1f52, 0x1f54, 0x1f56,
        0x1f80, 0x1f81, 0x1f82, 0x1f83, 0x1f84, 0x1f85, 0x1f86, 0x1f87,
        0x1f88, 0x1f89, 0x1f8a, 0x1f8b, 0x1f8c, 0x1f8d, 0x1f8e, 0x1f8f,
        0x1f90, 0x1f91, 0x1f92, 0x1f93, 0x1f94, 0x1f95, 0x1f96, 0x1f97,
        0x1f98, 0x1f99, 0x1f9a, 0x1f9b, 0x1f9c, 0x1f9d, 0x1f9e, 0x1f9f,
        0x1fa0, 0x1fa1, 0x1fa2, 0x1fa3, 0x1fa4, 0x1fa5, 0x1fa6, 0x1fa7,
        0x1fa8, 0x1fa9, 0x1faa, 0x1fab, 0x1fac, 0x1fad, 0x1fae, 0x1faf,
        0x1fb2, 0x1fb3, 0x1fb4, 0x1fb6, 0x1fb7, 0x1fbc, 0x1fc2, 0x1fc3,
        0x1fc4, 0x1fc6, 0x1fc7, 0x1fcc, 0x1fd2, 0x1fd3, 0x1fd6, 0x1fd7,
        0x1fe2, 0x1fe3, 0x1fe4, 0x1fe6, 0x1fe7, 0x1ff2, 0x1ff3, 0x1ff4,
        0x1ff6, 0x1ff7, 0x1ffc, 0xfb00, 0xfb01, 0xfb02, 0xfb03, 0xfb04,
        0xfb05, 0xfb06, 0xfb13, 0xfb14, 0xfb15, 0xfb16, 0xfb17};
        static StringOverridePropertyObject property_object(lc, 
                                                    SLC_ns::property_object, 
                                                    explicitly_defined_set, 
                                                    static_cast<const char *>(string_buffer), 
                                                    buffer_length, 
                                                    defined_cps);
    }
    namespace UC_ns {
        /** Code Point Ranges for uc (possibly overriding values from SUC)
        [00df, 00df], [0130, 0130], [0149, 0149], [01f0, 01f0],
        [0390, 0390], [03b0, 03b0], [0587, 0587], [1e96, 1e9a],
        [1f50, 1f50], [1f52, 1f52], [1f54, 1f54], [1f56, 1f56],
        [1f80, 1faf], [1fb2, 1fb4], [1fb6, 1fb7], [1fbc, 1fbc],
        [1fc2, 1fc4], [1fc6, 1fc7], [1fcc, 1fcc], [1fd2, 1fd3],
        [1fd6, 1fd7], [1fe2, 1fe4], [1fe6, 1fe7], [1ff2, 1ff4],
        [1ff6, 1ff7], [1ffc, 1ffc], [fb00, fb06], [fb13, fb17]**/

        const UnicodeSet explicitly_defined_set
                    {{{Empty, 6}, {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 4},
              {Mixed, 1}, {Empty, 12}, {Mixed, 2}, {Empty, 14}, {Mixed, 1},
              {Empty, 199}, {Mixed, 1}, {Empty, 5}, {Mixed, 1}, {Empty, 1},
              {Full, 1}, {Mixed, 3}, {Empty, 1752}, {Mixed, 1},
              {Empty, 32807}},
             {0x80000000, 0x00010000, 0x00000200, 0x00010000, 0x00010000,
              0x00010000, 0x00000080, 0x07c00000, 0x00550000, 0x10dcffff,
              0x00cc10dc, 0x10dc00dc, 0x00f8007f}};

        const unsigned buffer_length = 571;
        const static char __attribute__ ((aligned (32))) string_buffer[768] = u8R"__(SS
İ
ʼN
J̌
Ϊ́
Ϋ́
ԵՒ
H̱
T̈
W̊
Y̊
Aʾ
Υ̓
Υ̓̀
Υ̓́
Υ̓͂
ἈΙ
ἉΙ
ἊΙ
ἋΙ
ἌΙ
ἍΙ
ἎΙ
ἏΙ
ἈΙ
ἉΙ
ἊΙ
ἋΙ
ἌΙ
ἍΙ
ἎΙ
ἏΙ
ἨΙ
ἩΙ
ἪΙ
ἫΙ
ἬΙ
ἭΙ
ἮΙ
ἯΙ
ἨΙ
ἩΙ
ἪΙ
ἫΙ
ἬΙ
ἭΙ
ἮΙ
ἯΙ
ὨΙ
ὩΙ
ὪΙ
ὫΙ
ὬΙ
ὭΙ
ὮΙ
ὯΙ
ὨΙ
ὩΙ
ὪΙ
ὫΙ
ὬΙ
ὭΙ
ὮΙ
ὯΙ
ᾺΙ
ΑΙ
ΆΙ
Α͂
Α͂Ι
ΑΙ
ῊΙ
ΗΙ
ΉΙ
Η͂
Η͂Ι
ΗΙ
Ϊ̀
Ϊ́
Ι͂
Ϊ͂
Ϋ̀
Ϋ́
Ρ̓
Υ͂
Ϋ͂
ῺΙ
ΩΙ
ΏΙ
Ω͂
Ω͂Ι
ΩΙ
FF
FI
FL
FFI
FFL
ST
ST
ՄՆ
ՄԵ
ՄԻ
ՎՆ
ՄԽ
)__";

        const static std::vector<codepoint_t> defined_cps = {
        0x00df, 0x0130, 0x0149, 0x01f0, 0x0390, 0x03b0, 0x0587, 0x1e96,
        0x1e97, 0x1e98, 0x1e99, 0x1e9a, 0x1f50, 0x1f52, 0x1f54, 0x1f56,
        0x1f80, 0x1f81, 0x1f82, 0x1f83, 0x1f84, 0x1f85, 0x1f86, 0x1f87,
        0x1f88, 0x1f89, 0x1f8a, 0x1f8b, 0x1f8c, 0x1f8d, 0x1f8e, 0x1f8f,
        0x1f90, 0x1f91, 0x1f92, 0x1f93, 0x1f94, 0x1f95, 0x1f96, 0x1f97,
        0x1f98, 0x1f99, 0x1f9a, 0x1f9b, 0x1f9c, 0x1f9d, 0x1f9e, 0x1f9f,
        0x1fa0, 0x1fa1, 0x1fa2, 0x1fa3, 0x1fa4, 0x1fa5, 0x1fa6, 0x1fa7,
        0x1fa8, 0x1fa9, 0x1faa, 0x1fab, 0x1fac, 0x1fad, 0x1fae, 0x1faf,
        0x1fb2, 0x1fb3, 0x1fb4, 0x1fb6, 0x1fb7, 0x1fbc, 0x1fc2, 0x1fc3,
        0x1fc4, 0x1fc6, 0x1fc7, 0x1fcc, 0x1fd2, 0x1fd3, 0x1fd6, 0x1fd7,
        0x1fe2, 0x1fe3, 0x1fe4, 0x1fe6, 0x1fe7, 0x1ff2, 0x1ff3, 0x1ff4,
        0x1ff6, 0x1ff7, 0x1ffc, 0xfb00, 0xfb01, 0xfb02, 0xfb03, 0xfb04,
        0xfb05, 0xfb06, 0xfb13, 0xfb14, 0xfb15, 0xfb16, 0xfb17};
        static StringOverridePropertyObject property_object(uc, 
                                                    SUC_ns::property_object, 
                                                    explicitly_defined_set, 
                                                    static_cast<const char *>(string_buffer), 
                                                    buffer_length, 
                                                    defined_cps);
    }
    namespace TC_ns {
        /** Code Point Ranges for tc (possibly overriding values from STC)
        [00df, 00df], [0130, 0130], [0149, 0149], [01f0, 01f0],
        [0390, 0390], [03b0, 03b0], [0587, 0587], [1e96, 1e9a],
        [1f50, 1f50], [1f52, 1f52], [1f54, 1f54], [1f56, 1f56],
        [1f80, 1faf], [1fb2, 1fb4], [1fb6, 1fb7], [1fbc, 1fbc],
        [1fc2, 1fc4], [1fc6, 1fc7], [1fcc, 1fcc], [1fd2, 1fd3],
        [1fd6, 1fd7], [1fe2, 1fe4], [1fe6, 1fe7], [1ff2, 1ff4],
        [1ff6, 1ff7], [1ffc, 1ffc], [fb00, fb06], [fb13, fb17]**/

        const UnicodeSet explicitly_defined_set
                    {{{Empty, 6}, {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 4},
              {Mixed, 1}, {Empty, 12}, {Mixed, 2}, {Empty, 14}, {Mixed, 1},
              {Empty, 199}, {Mixed, 1}, {Empty, 5}, {Mixed, 1}, {Empty, 1},
              {Full, 1}, {Mixed, 3}, {Empty, 1752}, {Mixed, 1},
              {Empty, 32807}},
             {0x80000000, 0x00010000, 0x00000200, 0x00010000, 0x00010000,
              0x00010000, 0x00000080, 0x07c00000, 0x00550000, 0x10dcffff,
              0x00cc10dc, 0x10dc00dc, 0x00f8007f}};

        const unsigned buffer_length = 469;
        const static char __attribute__ ((aligned (32))) string_buffer[512] = u8R"__(Ss
İ
ʼN
J̌
Ϊ́
Ϋ́
Եւ
H̱
T̈
W̊
Y̊
Aʾ
Υ̓
Υ̓̀
Υ̓́
Υ̓͂
ᾈ
ᾉ
ᾊ
ᾋ
ᾌ
ᾍ
ᾎ
ᾏ
ᾈ
ᾉ
ᾊ
ᾋ
ᾌ
ᾍ
ᾎ
ᾏ
ᾘ
ᾙ
ᾚ
ᾛ
ᾜ
ᾝ
ᾞ
ᾟ
ᾘ
ᾙ
ᾚ
ᾛ
ᾜ
ᾝ
ᾞ
ᾟ
ᾨ
ᾩ
ᾪ
ᾫ
ᾬ
ᾭ
ᾮ
ᾯ
ᾨ
ᾩ
ᾪ
ᾫ
ᾬ
ᾭ
ᾮ
ᾯ
Ὰͅ
ᾼ
Άͅ
Α͂
ᾼ͂
ᾼ
Ὴͅ
ῌ
Ήͅ
Η͂
ῌ͂
ῌ
Ϊ̀
Ϊ́
Ι͂
Ϊ͂
Ϋ̀
Ϋ́
Ρ̓
Υ͂
Ϋ͂
Ὼͅ
ῼ
Ώͅ
Ω͂
ῼ͂
ῼ
Ff
Fi
Fl
Ffi
Ffl
St
St
Մն
Մե
Մի
Վն
Մխ
)__";

        const static std::vector<codepoint_t> defined_cps = {
        0x00df, 0x0130, 0x0149, 0x01f0, 0x0390, 0x03b0, 0x0587, 0x1e96,
        0x1e97, 0x1e98, 0x1e99, 0x1e9a, 0x1f50, 0x1f52, 0x1f54, 0x1f56,
        0x1f80, 0x1f81, 0x1f82, 0x1f83, 0x1f84, 0x1f85, 0x1f86, 0x1f87,
        0x1f88, 0x1f89, 0x1f8a, 0x1f8b, 0x1f8c, 0x1f8d, 0x1f8e, 0x1f8f,
        0x1f90, 0x1f91, 0x1f92, 0x1f93, 0x1f94, 0x1f95, 0x1f96, 0x1f97,
        0x1f98, 0x1f99, 0x1f9a, 0x1f9b, 0x1f9c, 0x1f9d, 0x1f9e, 0x1f9f,
        0x1fa0, 0x1fa1, 0x1fa2, 0x1fa3, 0x1fa4, 0x1fa5, 0x1fa6, 0x1fa7,
        0x1fa8, 0x1fa9, 0x1faa, 0x1fab, 0x1fac, 0x1fad, 0x1fae, 0x1faf,
        0x1fb2, 0x1fb3, 0x1fb4, 0x1fb6, 0x1fb7, 0x1fbc, 0x1fc2, 0x1fc3,
        0x1fc4, 0x1fc6, 0x1fc7, 0x1fcc, 0x1fd2, 0x1fd3, 0x1fd6, 0x1fd7,
        0x1fe2, 0x1fe3, 0x1fe4, 0x1fe6, 0x1fe7, 0x1ff2, 0x1ff3, 0x1ff4,
        0x1ff6, 0x1ff7, 0x1ffc, 0xfb00, 0xfb01, 0xfb02, 0xfb03, 0xfb04,
        0xfb05, 0xfb06, 0xfb13, 0xfb14, 0xfb15, 0xfb16, 0xfb17};
        static StringOverridePropertyObject property_object(tc, 
                                                    STC_ns::property_object, 
                                                    explicitly_defined_set, 
                                                    static_cast<const char *>(string_buffer), 
                                                    buffer_length, 
                                                    defined_cps);
    }
}


#endif