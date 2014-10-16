#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pabloAST.h>
#include <string>

namespace pablo {

class String : public PabloAST, public std::string {
    friend String * makeString(const std::string);
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::String;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~String(){

    }
    inline const std::string & getValue() const {
        return mValue;
    }
protected:
    String(const std::string string) noexcept
    : PabloAST(ClassTypeId::String)
    , std::string(string)
    {

    }
private:
    const std::string mValue;
};

inline String * makeString(const std::string string) {
    return new String(string);
}

}

#endif // PE_STRING_H
