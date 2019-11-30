/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TO_UTF8_H
#define TO_UTF8_H

#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>

namespace re {class CC;}
namespace re {

class EncodingTransformer : public RE_Transformer {
public:
    const cc::Alphabet * getIndexingAlphabet() const {return mIndexingAlphabet;}
    const cc::Alphabet * getEncodingAlphabet() const {return mEncodingAlphabet;}
protected:
    EncodingTransformer(std::string transformationName,
                        const cc::Alphabet * indexingAlphabet,
                        const cc::Alphabet * encodingAlphabet,
                        NameTransformationMode m = NameTransformationMode::None) :
    RE_Transformer(transformationName, m),
    mIndexingAlphabet(indexingAlphabet),
    mEncodingAlphabet(encodingAlphabet) {}
protected:
    const cc::Alphabet * mIndexingAlphabet;
    const cc::Alphabet * mEncodingAlphabet;
};

class UTF8_Transformer : public EncodingTransformer {
public:
    UTF8_Transformer(NameTransformationMode m = NameTransformationMode::None);
protected:
    RE * transformCC(CC * cc) override;
};

RE * toUTF8(RE * r, bool convertName = false);
}

#endif // TO_UTF8_H
