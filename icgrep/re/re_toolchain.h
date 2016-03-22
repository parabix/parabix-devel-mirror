/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TOOLCHAIN_H
#define RE_TOOLCHAIN_H

#include "utf_encoding.h"

#include <re/re_re.h>
#include <pablo/function.h>

re::RE * regular_expression_passes(const Encoding encoding, re::RE * re_ast);

pablo::PabloFunction * re2pablo_compiler(const Encoding encoding, re::RE * re_ast);

#endif
