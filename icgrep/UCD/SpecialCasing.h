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
#include "unicode_set.h"

namespace UCD {
    namespace LC_ns {
        /** Code Point Ranges for lc mapping to <none>
        **/

        const UnicodeSet null_codepoint_set
                    {{{Empty, 34816}},
             {}};

        /** Code Point Ranges for lc mapping to <codepoint>
        [0000, 012f], [0131, 1f87], [1f90, 1f97], [1fa0, 1fa7],
        [1fb0, 1fbb], [1fbd, 1fcb], [1fcd, 1ffb], [1ffd, 10ffff]**/
        const UnicodeSet reflexive_set
                    {{{Full, 9}, {Mixed, 1}, {Full, 242}, {Mixed, 4}, {Full, 34560}},
             {0xfffeffff, 0x00ff00ff, 0xefff00ff, 0xffffefff, 0xefffffff}};

        const unsigned buffer_length = 112;
        const static char __attribute__ ((aligned (32))) string_buffer[256] = u8R"__(i̇
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
ᾠ
ᾡ
ᾢ
ᾣ
ᾤ
ᾥ
ᾦ
ᾧ
ᾳ
ῃ
ῳ
)__";

        const static std::vector<codepoint_t> defined_cps = {
        0x0130, 0x1f88, 0x1f89, 0x1f8a, 0x1f8b, 0x1f8c, 0x1f8d, 0x1f8e,
        0x1f8f, 0x1f98, 0x1f99, 0x1f9a, 0x1f9b, 0x1f9c, 0x1f9d, 0x1f9e,
        0x1f9f, 0x1fa8, 0x1fa9, 0x1faa, 0x1fab, 0x1fac, 0x1fad, 0x1fae,
        0x1faf, 0x1fbc, 0x1fcc, 0x1ffc};
        static StringPropertyObject property_object(lc, 
                                                    null_codepoint_set, 
                                                    reflexive_set, 
                                                    static_cast<const char *>(string_buffer), 
                                                    buffer_length, 
                                                    defined_cps);
    }
    namespace UC_ns {
        /** Code Point Ranges for uc mapping to <none>
        **/

        const UnicodeSet null_codepoint_set
                    {{{Empty, 34816}},
             {}};

        /** Code Point Ranges for uc mapping to <codepoint>
        [0000, 00de], [00e0, 0148], [014a, 01ef], [01f1, 038f],
        [0391, 03af], [03b1, 0586], [0588, 1e95], [1e9b, 1f4f],
        [1f51, 1f51], [1f53, 1f53], [1f55, 1f55], [1f57, 1f7f],
        [1f88, 1f8f], [1f98, 1f9f], [1fa8, 1fb1], [1fb5, 1fb5],
        [1fb8, 1fc1], [1fc5, 1fc5], [1fc8, 1fd1], [1fd4, 1fd5],
        [1fd8, 1fe1], [1fe5, 1fe5], [1fe8, 1ff1], [1ff5, 1ff5],
        [1ff8, faff], [fb07, fb12], [fb18, 10ffff]**/
        const UnicodeSet reflexive_set
                    {{{Full, 6}, {Mixed, 1}, {Full, 3}, {Mixed, 1}, {Full, 4},
              {Mixed, 1}, {Full, 12}, {Mixed, 2}, {Full, 14}, {Mixed, 1},
              {Full, 199}, {Mixed, 1}, {Full, 5}, {Mixed, 1}, {Full, 1},
              {Mixed, 4}, {Full, 1752}, {Mixed, 1}, {Full, 32807}},
             {0x7fffffff, 0xfffffdff, 0xfffeffff, 0xfffeffff, 0xfffeffff,
              0xffffff7f, 0xf83fffff, 0xffaaffff, 0xff00ff00, 0xff23ff00,
              0xff33ff23, 0xff23ff23, 0xff07ff80}};

        const unsigned buffer_length = 358;
        const static char __attribute__ ((aligned (32))) string_buffer[512] = u8R"__(Ss
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
Ὰͅ
ᾼ
Άͅ
Α͂
ᾼ͂
Ὴͅ
ῌ
Ήͅ
Η͂
ῌ͂
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
        0x00df, 0x0149, 0x01f0, 0x0390, 0x03b0, 0x0587, 0x1e96, 0x1e97,
        0x1e98, 0x1e99, 0x1e9a, 0x1f50, 0x1f52, 0x1f54, 0x1f56, 0x1f80,
        0x1f81, 0x1f82, 0x1f83, 0x1f84, 0x1f85, 0x1f86, 0x1f87, 0x1f90,
        0x1f91, 0x1f92, 0x1f93, 0x1f94, 0x1f95, 0x1f96, 0x1f97, 0x1fa0,
        0x1fa1, 0x1fa2, 0x1fa3, 0x1fa4, 0x1fa5, 0x1fa6, 0x1fa7, 0x1fb2,
        0x1fb3, 0x1fb4, 0x1fb6, 0x1fb7, 0x1fc2, 0x1fc3, 0x1fc4, 0x1fc6,
        0x1fc7, 0x1fd2, 0x1fd3, 0x1fd6, 0x1fd7, 0x1fe2, 0x1fe3, 0x1fe4,
        0x1fe6, 0x1fe7, 0x1ff2, 0x1ff3, 0x1ff4, 0x1ff6, 0x1ff7, 0xfb00,
        0xfb01, 0xfb02, 0xfb03, 0xfb04, 0xfb05, 0xfb06, 0xfb13, 0xfb14,
        0xfb15, 0xfb16, 0xfb17};
        static StringPropertyObject property_object(uc, 
                                                    null_codepoint_set, 
                                                    reflexive_set, 
                                                    static_cast<const char *>(string_buffer), 
                                                    buffer_length, 
                                                    defined_cps);
    }
    namespace TC_ns {
        /** Code Point Ranges for tc mapping to <none>
        **/

        const UnicodeSet null_codepoint_set
                    {{{Empty, 34816}},
             {}};

        /** Code Point Ranges for tc mapping to <codepoint>
        [0000, 00de], [00e0, 0148], [014a, 01ef], [01f1, 038f],
        [0391, 03af], [03b1, 0586], [0588, 1e95], [1e9b, 1f4f],
        [1f51, 1f51], [1f53, 1f53], [1f55, 1f55], [1f57, 1f7f],
        [1fb0, 1fb1], [1fb5, 1fb5], [1fb8, 1fbb], [1fbd, 1fc1],
        [1fc5, 1fc5], [1fc8, 1fcb], [1fcd, 1fd1], [1fd4, 1fd5],
        [1fd8, 1fe1], [1fe5, 1fe5], [1fe8, 1ff1], [1ff5, 1ff5],
        [1ff8, 1ffb], [1ffd, faff], [fb07, fb12], [fb18, 10ffff]**/
        const UnicodeSet reflexive_set
                    {{{Full, 6}, {Mixed, 1}, {Full, 3}, {Mixed, 1}, {Full, 4},
              {Mixed, 1}, {Full, 12}, {Mixed, 2}, {Full, 14}, {Mixed, 1},
              {Full, 199}, {Mixed, 1}, {Full, 5}, {Mixed, 1}, {Full, 1},
              {Empty, 1}, {Mixed, 3}, {Full, 1752}, {Mixed, 1},
              {Full, 32807}},
             {0x7fffffff, 0xfffffdff, 0xfffeffff, 0xfffeffff, 0xfffeffff,
              0xffffff7f, 0xf83fffff, 0xffaaffff, 0xef230000, 0xff33ef23,
              0xef23ff23, 0xff07ff80}};

        const unsigned buffer_length = 568;
        const static char __attribute__ ((aligned (32))) string_buffer[768] = u8R"__(SS
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
        0x00df, 0x0149, 0x01f0, 0x0390, 0x03b0, 0x0587, 0x1e96, 0x1e97,
        0x1e98, 0x1e99, 0x1e9a, 0x1f50, 0x1f52, 0x1f54, 0x1f56, 0x1f80,
        0x1f81, 0x1f82, 0x1f83, 0x1f84, 0x1f85, 0x1f86, 0x1f87, 0x1f88,
        0x1f89, 0x1f8a, 0x1f8b, 0x1f8c, 0x1f8d, 0x1f8e, 0x1f8f, 0x1f90,
        0x1f91, 0x1f92, 0x1f93, 0x1f94, 0x1f95, 0x1f96, 0x1f97, 0x1f98,
        0x1f99, 0x1f9a, 0x1f9b, 0x1f9c, 0x1f9d, 0x1f9e, 0x1f9f, 0x1fa0,
        0x1fa1, 0x1fa2, 0x1fa3, 0x1fa4, 0x1fa5, 0x1fa6, 0x1fa7, 0x1fa8,
        0x1fa9, 0x1faa, 0x1fab, 0x1fac, 0x1fad, 0x1fae, 0x1faf, 0x1fb2,
        0x1fb3, 0x1fb4, 0x1fb6, 0x1fb7, 0x1fbc, 0x1fc2, 0x1fc3, 0x1fc4,
        0x1fc6, 0x1fc7, 0x1fcc, 0x1fd2, 0x1fd3, 0x1fd6, 0x1fd7, 0x1fe2,
        0x1fe3, 0x1fe4, 0x1fe6, 0x1fe7, 0x1ff2, 0x1ff3, 0x1ff4, 0x1ff6,
        0x1ff7, 0x1ffc, 0xfb00, 0xfb01, 0xfb02, 0xfb03, 0xfb04, 0xfb05,
        0xfb06, 0xfb13, 0xfb14, 0xfb15, 0xfb16, 0xfb17};
        static StringPropertyObject property_object(tc, 
                                                    null_codepoint_set, 
                                                    reflexive_set, 
                                                    static_cast<const char *>(string_buffer), 
                                                    buffer_length, 
                                                    defined_cps);
    }
}


#endif
