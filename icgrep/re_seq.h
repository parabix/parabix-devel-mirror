/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef JOIN_H
#define JOIN_H

#include "re_re.h"
#include <list>

class Seq : public RE
{
public:
    Seq();
    Seq(std::list<RE*>* lst);
    ~Seq();
    std::list<RE*>* GetREList();
    void AddREListItem(RE *re);
private:
    std::list<RE*>* mList;
};

#endif // JOIN_H




