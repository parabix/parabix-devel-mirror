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

class UTF_Encoding
{
public:
    UTF_Encoding();
    //UTF_Encoding(std::string name, int bits, uint32_t mask);
    std::string getName();
    void setName(std::string name);
    int getBits();
    void setBits(int bits);
    uint32_t getMask();
    void setMask(uint32_t mask);
    bool getDefault();
    void setDefault(bool defaultEncoding);
    bool getLocked();
    void setLocked(bool locked);
    bool getEncodingError();
    void setEncodingError(bool error);
    std::string getBasisPattern(int n);
    void setBasisPattern(std::string pattern);
    std::vector<std::string> getBasisPatternVector();
    void setBasisPatternVector(std::vector<std::string> basisPattern);
private:
    std::string mName;
    int mBits;
    uint32_t mMask;
    bool mDefault;
    bool mLocked;
    bool mEncodingError;
    std::vector<std::string> mBasisPattern;
};

#endif // UTF_ENCODING_H
