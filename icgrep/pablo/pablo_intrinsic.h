/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_INTRINSIC_H
#define PABLO_INTRINSIC_H

#include <initializer_list>
#include <pablo/pabloAST.h>

namespace pablo {

enum class Intrinsic {
    InclusiveSpan,
    PrintRegister
};

class IntrinsicCall final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(PabloAST const * e) {
        return e->getClassTypeId() == ClassTypeId::IntrinsicCall;
    }
    static inline bool classof(void const *) {
        return false;
    }
    virtual ~IntrinsicCall() {}

    inline Intrinsic getIntrinsic() const noexcept {
        return mIntrinsic;
    }

    inline std::vector<PabloAST *> const & getArgv() const noexcept {
        return mArgv;
    }

protected:
    IntrinsicCall(Intrinsic intrinsic, llvm::Type * type, std::vector<PabloAST *> argv, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::IntrinsicCall, type, argv, name, allocator)
    , mIntrinsic(intrinsic)
    , mArgv(std::move(argv))
    {}

    Intrinsic               mIntrinsic;
    std::vector<PabloAST *> mArgv;
};

} // namespace pablo

#endif // PABLO_INTRINSIC_H
