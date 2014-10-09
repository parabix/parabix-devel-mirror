#ifndef PE_CALL_H
#define PE_CALL_H

#include <pablo/pe_pabloe.h>
#include <pablo/pe_string.h>

namespace pablo {

class Call : public PabloE {
    friend class PabloBlock;
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
    Call(const PabloE * callee)
    : PabloE(ClassTypeId::Call)
    , mCallee(cast<String>(callee)) {

    }
private:
    const String * const mCallee;
};
}

#endif // PE_CALL_H


