#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pe_pabloe.h>
#include <string>

namespace pablo {

class String : public PabloE, public std::string {
    friend String * makeString(const std::string);
public:
    static inline bool classof(const PabloE * e) {
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
    : PabloE(ClassTypeId::String)
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
