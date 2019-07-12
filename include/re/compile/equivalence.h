/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_EQUIVALENCE_H
#define RE_EQUIVALENCE_H

#include <string>
#include <locale>
#include <codecvt>
#include <re/compile/re_toolchain.h>
#include <ucd/core/unicode_set.h>
#include <ucd/data/Equivalence.h>

namespace re { class RE; class CC; class Seq; class Group;}
namespace UCD { class EnumeratedPropertyObject; class StringPropertyObject;}

namespace UCD {

re::RE * addClusterMatches(re::RE * r, UCD::EquivalenceOptions options = UCD::Canonical);

re::RE * addEquivalentCodepoints(re::RE * r, UCD::EquivalenceOptions options = UCD::Canonical);

}
#endif
