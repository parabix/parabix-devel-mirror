#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pabloAST.h>
#include <string>

namespace pablo {

class String : public PabloAST {
    friend class SymbolGenerator;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::String;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~String(){

    }
    inline const std::string & str() const {
        return mValue;
    }
    inline std::string str() {
        return mValue;
    }
    inline bool isGenerated() const {
        return mGenerated;
    }
    inline bool isUserDefined() const {
        return !mGenerated;
    }
protected:
    String(const std::string && value, const bool generated) noexcept
    : PabloAST(ClassTypeId::String)
    , mValue(value)
    , mGenerated(generated)
    {

    }
private:
    const std::string   mValue;
    const bool          mGenerated;
};

}

#endif // PE_STRING_H
