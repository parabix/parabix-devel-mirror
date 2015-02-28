/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF_ENCODING_H
#define UTF_ENCODING_H

#include <stdint.h>
#include <string>
#include <vector>

class Encoding{
public:

    enum class Type {
        ASCII
        , UTF_8
    };

    Encoding(Type type, unsigned bits);
    Encoding(Type type, unsigned bits, unsigned mask);
    Type getType() const;
    unsigned getBits() const;
    unsigned getMask() const;
private:
    Type        mType;
    unsigned    mBits;
    unsigned    mMask;
};

inline Encoding::Encoding(Type type, unsigned bits)
: mType(type)
, mBits(bits)
, mMask((static_cast<unsigned>(1) << bits) - static_cast<unsigned>(1))
{

}

inline Encoding::Encoding(Type type, unsigned bits, unsigned mask)
: mType(type)
, mBits(bits)
, mMask(mask)
{

}

inline Encoding::Type Encoding::getType() const {
    return mType;
}

inline unsigned Encoding::getBits() const {
    return mBits;
}

inline unsigned Encoding::getMask() const {
    return mMask;
}

#endif // UTF_ENCODING_H
