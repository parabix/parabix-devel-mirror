#ifndef RE_NAME_H
#define RE_NAME_H

#include "re_re.h"

#include <string>

class Name : public RE
{
public:
    Name(std::string name);
    std::string getName();
    ~Name();
private:
    std::string mName;
};

#endif // RE_NAME_H
