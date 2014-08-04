/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef JOIN_H
#define JOIN_H

#include "re_re.h"
#include "re_cc.h"
#include "re_name.h"
#include <list>
#include <sstream>
#include <utility>

class Seq : public RE
{
public:
    typedef enum {Normal,Byte} Type;
    Seq();
    Seq(std::list<RE*>* lst);
    Seq(std::list<RE*> lst);
    ~Seq();
    std::list<RE*>* GetREList();
    void AddREListItem(RE *re);
    std::string getName();
    Type getType();
    void setType(Type type);
private:
    std::list<RE*>* mList;
    Type mType;
};

#endif // JOIN_H




