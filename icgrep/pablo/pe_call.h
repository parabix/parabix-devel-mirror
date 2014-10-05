#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pe_pabloe.h>
#include <pablo/pe_string.h>

namespace pablo {

class Call : public PabloE {
    friend Call * makeCall(const String *);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Call;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Call() {

    }
    inline const std::string & getCallee() const {
        return *mCallee;
    }
protected:
    Call(const String * callee)
    : PabloE(ClassTypeId::Call)
    , mCallee(callee) {

    }
private:
    const String * const mCallee;
};

inline Call * makeCall(const String * callee) {
    return new Call(callee);
}

}

#endif // PE_CALL_H


