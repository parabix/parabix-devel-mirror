#ifndef PE_CALL_H
#define PE_CALL_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class Call : public PabloE {
public:
    Call(std::string callee)
    : PabloE(ClassTypeId::Call)
    , mCallee(callee) {

    }

    virtual ~Call() {

    }

    inline const std::string & getCallee() const {
        return mCallee;
    }
private:
    std::string mCallee;
};

}

#endif // PE_CALL_H


