/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_INTRINSIC_H
#define PABLO_INTRINSIC_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/ArrayRef.h>
#include <llvm/ADT/StringRef.h>

namespace pablo {

enum class Intrinsic {
    SpanUpTo,
    InclusiveSpan,
    ExclusiveSpan,
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

    Intrinsic getIntrinsic() const noexcept {
        return mIntrinsic;
    }

    llvm::ArrayRef<PabloAST *> getArgv() const noexcept {
        return llvm::ArrayRef<PabloAST *>(mOperand, mOperands);
    }

    inline llvm::StringRef getIntrinsicName() const noexcept {
        #define CASE(INTRINSIC) case Intrinsic::INTRINSIC: return #INTRINSIC
        switch (mIntrinsic) {
            CASE(InclusiveSpan);
            CASE(PrintRegister);
        default:
            llvm_unreachable("unexpected intrinsic");
        }
        #undef CASE
    }

    inline bool isCarryProducing() const noexcept {
        switch (mIntrinsic) {
            case Intrinsic::SpanUpTo:
            case Intrinsic::InclusiveSpan:
            case Intrinsic::ExclusiveSpan:
                return true;
            default:
                return false;
        }
    }

    inline bool isAdvanceType() const noexcept {
        return false;
    }

protected:
    IntrinsicCall(Intrinsic intrinsic, llvm::Type * type, llvm::ArrayRef<PabloAST *> argv, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::IntrinsicCall, type, argv, name, allocator)
    , mIntrinsic(intrinsic)
    {}

    const Intrinsic mIntrinsic;

};

} // namespace pablo

#endif // PABLO_INTRINSIC_H
