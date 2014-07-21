/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"
#include <list>

class Alt : public RE
{
public:
    Alt();
    Alt(std::list<RE*>* lst);
    ~Alt();
    std::list<RE*>* GetREList();
    void AddREListItem(RE *re);
private:
    std::list<RE*>* mList;
};

#endif // ALT_H

