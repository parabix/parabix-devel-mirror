/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

namespace re {

class RE;

RE * resolveModesAndExternalSymbols(RE * r, bool globallyCaseInsensitive = false);

RE * excludeUnicodeLineBreak(RE * r);

RE * regular_expression_passes(RE * r);

}
