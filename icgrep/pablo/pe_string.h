#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pabloAST.h>
#include <string>

namespace pablo {

class String : public PabloAST {
    friend String * makeString(const std::string value) noexcept;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::String;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~String(){

    }
    virtual PabloAST * getOperand(const unsigned) const {
        assert (false);
        return nullptr;
    }
    virtual unsigned getNumOperands() const {
        return 0;
    }
    virtual void setOperand(const unsigned, PabloAST *) {
        assert (false);
    }
    inline const std::string & str() const {
        return mValue;
    }
    inline std::string str() {
        return mValue;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    String(const std::string && value) noexcept
    : PabloAST(ClassTypeId::String)
    , mValue(value)
    {

    }
private:
    const std::string mValue;
};

inline String * makeString(const std::string value) noexcept {
    return new String(std::move(value));
}


}

#endif // PE_STRING_H
