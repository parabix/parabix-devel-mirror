/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

namespace re {
    class RE;
}

namespace grep {

/**
 * Fully resolves all unicode names in a given regular expression including
 * names who's values are regular expressions.
 * 
 * This is in contrast to re/unicode/re_name_resolve.h's re::resolveUnicodeNames
 * which cannot resolve names with regular expressions as values due to that
 * functionality relying on grep::GrepEngine.
 */
re::RE * resolveUnicodeNames(re::RE * re);

}
