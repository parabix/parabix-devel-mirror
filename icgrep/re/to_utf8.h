/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TO_UTF8_H
#define TO_UTF8_H

#include <re/re_re.h>
#include <re/re_toolchain.h>

namespace re {
class CC;

class UTF8_Transformer : public RE_Transformer {
public:
    UTF8_Transformer(NameTransformationMode m = NameTransformationMode::None) : RE_Transformer("ToUTF8", m) {}
    RE * transformCC(CC * cc) override;
};

inline RE * toUTF8(RE * r, bool convertName = false) {
    return UTF8_Transformer(convertName ? NameTransformationMode::TransformDefinition : NameTransformationMode::None ).transformRE(r);}
}
#endif // TO_UTF8_H
