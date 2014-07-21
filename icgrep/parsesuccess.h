/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PARSESUCCESS_H
#define PARSESUCCESS_H

#include "parseresult.h"
#include "re_re.h"

class ParseSuccess : public ParseResult
{
public:
    ParseSuccess(RE* re);
    ~ParseSuccess();
    RE* getRE();
private:
    RE* mRE;
};

#endif // PARSESUCCESS_H

