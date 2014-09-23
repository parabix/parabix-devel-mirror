#ifndef RE_NAME_H
#define RE_NAME_H

#include "re_re.h"

#include <string>

class Name : public RE
{
public:
    typedef enum {FixedLength,Unicode,UnicodeCategory} Type;
    Name();
    Name(const Name * name);
    Name(std::string name);
    void setName(std::string name);
    std::string getName() const;
    void setNegated(const bool is_negated);
    bool isNegated() const;
    void setType(const Type type);
    Type getType() const;
    ~Name();
private:
    std::string mName;
    bool mNegated;
    Type mType;
};

#endif // RE_NAME_H
