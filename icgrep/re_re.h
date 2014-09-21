/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_H
#define RE_H

#include <vector>

class RE
{
public:
    typedef std::vector<RE*>            Vector;
    virtual ~RE();
protected:
    RE();
};

#endif // RE_H


