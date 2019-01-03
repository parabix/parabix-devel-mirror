/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_CONTEXTUAL_SIMPLIFICATION_H
#define RE_CONTEXTUAL_SIMPLIFICATION_H

#include <re/re_toolchain.h>

namespace re {

RE * simplifyAssertions(RE * r);
}

#endif
