/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PARSEFAILURE_H
#define PARSEFAILURE_H

#include "parseresult.h"
#include <string>

class ParseFailure : public ParseResult
{
public:
    ParseFailure(std::string msg);
    ~ParseFailure();
    std::string getErrorMsg();
private:
    std::string mErrorMsg;
};

#endif // PARSEFAILURE_H
