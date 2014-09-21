/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"
#include <algorithm>
#include <list>


class Alt : public RE, public RE::Vector {
public:
    typedef RE::Vector Vector;
    Alt();    
    Alt(iterator begin, iterator end);
    virtual ~Alt();
};

#endif // ALT_H

