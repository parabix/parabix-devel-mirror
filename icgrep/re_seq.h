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

class Seq : public RE, public RE::Vector {
public:
    typedef RE::Vector Vector;
    typedef enum {
        Normal,
        Byte
    } Type;
    Seq();
    Seq(const Type type);
    Seq(const Type type, iterator begin, iterator end);
    virtual ~Seq();
    std::string getName() const;
    Type getType() const;
    void setType(Type type);
private:
    Type    mType;
};

#endif // JOIN_H




