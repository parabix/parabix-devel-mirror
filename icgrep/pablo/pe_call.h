#ifndef PE_CALL_H
#define PE_CALL_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class Call : public PabloE {
    friend Call * make_call(const std::string callee);
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
        return mCallee;
    }
protected:
    Call(const std::string callee)
    : PabloE(ClassTypeId::Call)
    , mCallee(callee) {

    }
private:
    const std::string mCallee;
};

inline Call * make_call(const std::string callee) {
    return new Call(callee);
}

}

#endif // PE_CALL_H


