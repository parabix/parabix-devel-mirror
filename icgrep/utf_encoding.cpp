/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "utf_encoding.h"

UTF_Encoding::UTF_Encoding()
{
    mEncodingError = false;
    mDefault = true;
    mLocked = false;
}

std::string UTF_Encoding::getName()
{
    return mName;
}

void UTF_Encoding::setName(std::string name)
{
    mName = name;
}

int UTF_Encoding::getBits()
{
    return mBits;
}

void UTF_Encoding::setBits(int bits)
{
    mBits = bits;
}

uint32_t UTF_Encoding::getMask()
{
    return mMask;
}

void UTF_Encoding::setMask(uint32_t mask)
{
    mMask = mask;
}

bool UTF_Encoding::getDefault()
{
    return mDefault;
}

void UTF_Encoding::setDefault(bool defaultEncoding)
{
    mDefault = defaultEncoding;
}

bool UTF_Encoding::getLocked()
{
    return mLocked;
}

void UTF_Encoding::setLocked(bool locked)
{
    mLocked = locked;
}

bool UTF_Encoding::getEncodingError()
{
    return mEncodingError;
}

void UTF_Encoding::setEncodingError(bool error)
{
    mEncodingError= error;
}

void UTF_Encoding::setBasisPattern(std::string pattern)
{
    mBasisPattern.push_back(pattern);
}

std::string UTF_Encoding::getBasisPattern(int n)
{
    return mBasisPattern.at(n);
}

std::vector<std::string> UTF_Encoding::getBasisPatternVector()
{
    return mBasisPattern;
}


