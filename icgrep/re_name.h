#ifndef RE_NAME_H
#define RE_NAME_H

#include "re_re.h"

#include <string>

class Name : public RE
{
public:
    typedef enum {FixedLength,Unicode,UnicodeCategory} Type;
    Name();
    Name(std::string name);
    void setName(std::string name);
    std::string getName();
    void setType(Type type);
    Type getType();
    ~Name();
private:
    std::string mName;
    Type mType;
};

#endif // RE_NAME_H
