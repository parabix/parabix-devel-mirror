#include <pablo/builder.hpp>

#include <pablo/boolean.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_repeat.h>
#include <pablo/pe_pack.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_count.h>
#include <pablo/pe_debugprint.h>
#include <pablo/pe_everynth.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_terminate.h>
#include <boost/preprocessor/variadic/elem.hpp>
#include <type_traits>

using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

#define GET(I, ...) \
    reinterpret_cast<decltype(BOOST_PP_VARIADIC_ELEM(I, __VA_ARGS__))>(arg##I)

#define MAKE_UNARY(TYPE, ...) \
[&](){ /* immediately invoked lambda */ \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__)); \
    } \
    inline __##NAME(PabloBlock * const pb) : mPb(pb) {} \
private: \
    PabloBlock * const mPb; \
}; \
__##NAME functor(mPb); \
return cast<TYPE>(mExprTable.findUnaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_NAMED_UNARY(TYPE, PREFIX, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), mPrefix); \
    } \
    inline __##NAME(PabloBlock * const pb, const llvm::StringRef prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * const mPb; \
    const llvm::StringRef mPrefix; \
}; \
__##NAME functor(mPb, prefix); \
return cast<TYPE>(mExprTable.findUnaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_BINARY(TYPE, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0, void * const arg1) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), GET(1, __VA_ARGS__)); \
    } \
    inline __##NAME(PabloBlock * const pb) : mPb(pb) {} \
private: \
    PabloBlock * const mPb; \
}; \
__##NAME functor(mPb); \
return cast<TYPE>(mExprTable.findBinaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_NAMED_BINARY(TYPE, PREFIX, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0, void * const arg1) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), GET(1, __VA_ARGS__), mPrefix); \
    } \
    inline __##NAME(PabloBlock * const pb, const llvm::StringRef prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * const mPb; \
    const llvm::StringRef mPrefix; \
}; \
__##NAME functor(mPb, PREFIX); \
return cast<TYPE>(mExprTable.findBinaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_TERNARY(TYPE, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0, void * const arg1, void * const arg2) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), GET(1, __VA_ARGS__), GET(2, __VA_ARGS__)); \
    } \
    inline __##NAME(PabloBlock * const pb) : mPb(pb) {} \
private: \
    PabloBlock * const mPb; \
}; \
__##NAME functor(mPb); \
return cast<TYPE>(mExprTable.findTernaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_NAMED_TERNARY(TYPE, PREFIX, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0, void * const arg1, void * const arg2) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), GET(1, __VA_ARGS__), GET(2, __VA_ARGS__), mPrefix); \
    } \
    inline __##NAME(PabloBlock * const pb, const llvm::StringRef prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * const mPb; \
    const llvm::StringRef mPrefix; \
}; \
__##NAME functor(mPb, PREFIX); \
return cast<TYPE>(mExprTable.findTernaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_QUATERNARY(TYPE, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0, void * const arg1, void * const arg2, void * const arg3) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), GET(1, __VA_ARGS__), GET(2, __VA_ARGS__), GET(3, __VA_ARGS__)); \
    } \
    inline __##NAME(PabloBlock * const pb) : mPb(pb) {} \
private: \
    PabloBlock * const mPb; \
}; \
__##NAME functor(mPb); \
return cast<TYPE>(mExprTable.findQuaternaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()

#define MAKE_NAMED_QUATERNARY(TYPE, PREFIX, ...) \
[&](){ /* immediately invoked lambda */  \
struct __##NAME { \
    inline PabloAST * operator()(void * const arg0, void * const arg1, void * const arg2, void * const arg3) { \
        return mPb->create##TYPE(GET(0, __VA_ARGS__), GET(1, __VA_ARGS__), GET(2, __VA_ARGS__), GET(3, __VA_ARGS__), mPrefix); \
    } \
    inline __##NAME(PabloBlock * const pb, const llvm::StringRef prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * const mPb; \
    const llvm::StringRef mPrefix; \
}; \
__##NAME functor(mPb, PREFIX); \
return cast<TYPE>(mExprTable.findQuaternaryOrCall(std::move(functor), TypeId::TYPE, __VA_ARGS__)); \
}()


PabloAST * PabloBuilder::createAdvance(PabloAST * expr, not_null<Integer *> shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount.get())->value() == 0) {
        return expr;
    }
    return MAKE_BINARY(Advance, expr, shiftAmount.get());
}

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, not_null<Integer *> shiftAmount, const llvm::StringRef prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount.get())->value() == 0) {
        return expr;
    }
    return MAKE_NAMED_BINARY(Advance, prefix, expr, shiftAmount.get());
}

PabloAST * PabloBuilder::createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, not_null<Integer *> shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount.get())->value() == 0) {
        return expr;
    }
    else if (indexStream == nullptr || isa<Ones>(indexStream)) {
        return createAdvance(expr, shiftAmount);
    }
    else if (cast<Integer>(shiftAmount.get())->value() == 1) {
        return createAdvanceThenScanTo(createAnd(expr, indexStream), indexStream);
    }
    return MAKE_TERNARY(IndexedAdvance, expr, indexStream, shiftAmount.get());
}

PabloAST * PabloBuilder::createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, not_null<Integer *> shiftAmount, const llvm::StringRef prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount.get())->value() == 0) {
        return expr;
    }
    else if (indexStream == nullptr || isa<Ones>(indexStream)) {
        return createAdvance(expr, shiftAmount, prefix);
    }
    else if (cast<Integer>(shiftAmount.get())->value() == 1) {
        return createAdvanceThenScanTo(expr, indexStream, prefix);
    }
    return MAKE_NAMED_TERNARY(IndexedAdvance, prefix, expr, indexStream, shiftAmount.get());
}

Extract * PabloBuilder::createExtract(Var * array, not_null<Integer *> index) {
    return MAKE_BINARY(Extract, array, index.get());
}

PabloAST * PabloBuilder::createLookahead(PabloAST * expr, not_null<Integer *> shiftAmount) {
    if (LLVM_UNLIKELY(isa<Zeroes>(expr) || cast<Integer>(shiftAmount.get())->value() == 0)) {
        return expr;
    }
    return MAKE_BINARY(Lookahead, expr, shiftAmount.get());
}

PabloAST * PabloBuilder::createLookahead(PabloAST * expr, not_null<Integer *> shiftAmount, const llvm::StringRef prefix) {
    if (LLVM_UNLIKELY(isa<Zeroes>(expr) || cast<Integer>(shiftAmount.get())->value() == 0)) {
        return expr;
    }
    return MAKE_NAMED_BINARY(Lookahead, prefix, expr, shiftAmount.get());
}

PabloAST * PabloBuilder::createNot(PabloAST * expr) {
    if (LLVM_UNLIKELY(isa<Ones>(expr))) {
        return createZeroes(expr->getType());
    }
    else if (LLVM_UNLIKELY(isa<Zeroes>(expr))){
        return createOnes(expr->getType());
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {
        return not1->getOperand(0);
    } else if (Ternary * ternary = dyn_cast<Ternary>(expr)) {
        const auto mask = ternary->getMask()->value();
        const auto negated_mask = mask ^ 0xFF;
        return createTernary(negated_mask, ternary->getA(), ternary->getB(), ternary->getC());
    }
    return MAKE_UNARY(Not, expr);
}

PabloAST * PabloBuilder::createNot(PabloAST * expr, const llvm::StringRef prefix) {
    if (LLVM_UNLIKELY(isa<Ones>(expr))) {
        return createZeroes(expr->getType());
    }
    else if (LLVM_UNLIKELY(isa<Zeroes>(expr))){
        return createOnes(expr->getType());
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {
        return not1->getOperand(0);
    } else if (Ternary * ternary = dyn_cast<Ternary>(expr)) {
        const auto mask = ternary->getMask()->value();
        const auto negated_mask = mask ^ 0xFF;
        return createTernary(negated_mask, ternary->getA(), ternary->getB(), ternary->getC());
    }
    return MAKE_NAMED_UNARY(Not, prefix, expr);
}

PabloAST * PabloBuilder::createCount(PabloAST * expr) {
    return MAKE_UNARY(Count, expr);
}

PabloAST * PabloBuilder::createCount(PabloAST * expr, const llvm::StringRef prefix) {
    return MAKE_NAMED_UNARY(Count, prefix, expr);
}

PabloAST * PabloBuilder::createEveryNth(PabloAST * expr, not_null<Integer *> n) {
    return MAKE_BINARY(EveryNth, expr, n.get());
}

PabloAST * PabloBuilder::createEveryNth(PabloAST * expr, not_null<Integer *> n, const llvm::StringRef prefix) {
    return MAKE_NAMED_BINARY(EveryNth, prefix, expr, n.get());
}

PabloAST * PabloBuilder::createRepeat(not_null<Integer *> fieldWidth, PabloAST * value) {
    return MAKE_BINARY(Repeat, fieldWidth.get(), value);
}

PabloAST * PabloBuilder::createRepeat(not_null<Integer *> fieldWidth, PabloAST * value, const llvm::StringRef prefix) {
    return MAKE_NAMED_BINARY(Repeat, prefix, fieldWidth.get(), value);
}


PabloAST * PabloBuilder::createPackL(not_null<Integer *> fieldWidth, PabloAST * value) {
    return MAKE_BINARY(PackL, fieldWidth.get(), value);
}

PabloAST * PabloBuilder::createPackH(not_null<Integer *> fieldWidth, PabloAST * value) {
    return MAKE_BINARY(PackH, fieldWidth.get(), value);
}

PabloAST * PabloBuilder::createDebugPrint(PabloAST * expr) {
    if (LLVM_UNLIKELY(isa<DebugPrint>(expr) || isa<Zeroes>(expr))) {
        return expr;
    }
    return MAKE_UNARY(DebugPrint, expr);
}

PabloAST * PabloBuilder::createDebugPrint(PabloAST * expr, const llvm::StringRef prefix) {
    if (LLVM_UNLIKELY(isa<DebugPrint>(expr) || isa<Zeroes>(expr))) {
        return expr;
    }
    return MAKE_NAMED_UNARY(DebugPrint, prefix, expr);
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    } else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getOperand(0), not2->getOperand(0)));
        } else if (equals(not1->getOperand(0), expr2)) {
            return createZeroes(expr1->getType());
        }
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getOperand(0))) {
            return createZeroes(expr1->getType());
        }
    } else if (Or * or1 = dyn_cast<Or>(expr1)) {
        if (equals(or1->getOperand(0), expr2) || equals(or1->getOperand(1), expr2)) {
            return expr2;
        }
    } else if (Or * or2 = dyn_cast<Or>(expr2)) {
        if (equals(or2->getOperand(0), expr1) || equals(or2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    return MAKE_BINARY(And, expr1, expr2);
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix) {
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    } else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getOperand(0), not2->getOperand(0)), prefix);
        } else if (equals(not1->getOperand(0), expr2)) {
            return createZeroes(expr1->getType());
        }
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getOperand(0))) {
            return createZeroes(expr1->getType());
        }
    } else if (Or * or1 = dyn_cast<Or>(expr1)) {
        if (equals(or1->getOperand(0), expr2) || equals(or1->getOperand(1), expr2)) {
            return expr2;
        }
    } else if (Or * or2 = dyn_cast<Or>(expr2)) {
        if (equals(or2->getOperand(0), expr1) || equals(or2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    return MAKE_NAMED_BINARY(And, prefix, expr1, expr2);
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a ∨ b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getOperand(0), createNot(expr2)));
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b ∨ a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getOperand(0), createNot(expr1)));
    } else if (equals(expr1, expr2)) {
        return expr1;
    } else if (And * and1 = dyn_cast<And>(expr1)) {
        PabloAST * const expr1a = and1->getOperand(0);
        PabloAST * const expr1b = and1->getOperand(1);
        if (And * and2 = dyn_cast<And>(expr2)) {
            PabloAST * const expr2a = and2->getOperand(0);
            PabloAST * const expr2b = and2->getOperand(1);
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b));
            } else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a));
            } else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a));
            } else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b));
            }
        } else if (equals(expr1a, expr2) || equals(expr1b, expr2)) {
            // (a ∧ b) ∨ a = a
            return expr2;
        }
    } else if (And * and2 = dyn_cast<And>(expr2)) {
        if (equals(and2->getOperand(0), expr1) || equals(and2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    return MAKE_BINARY(Or, expr1, expr2);
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix) {
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a ∨ b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getOperand(0), createNot(expr2)), prefix);
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b ∨ a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getOperand(0), createNot(expr1)), prefix);
    } else if (equals(expr1, expr2)) {
        return expr1;
    } else if (And * and1 = dyn_cast<And>(expr1)) {
        PabloAST * const expr1a = and1->getOperand(0);
        PabloAST * const expr1b = and1->getOperand(1);
        if (And * and2 = dyn_cast<And>(expr2)) {
            PabloAST * const expr2a = and2->getOperand(0);
            PabloAST * const expr2b = and2->getOperand(1);
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b), prefix);
            } else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a), prefix);
            } else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a), prefix);
            } else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b), prefix);
            }
        } else if (equals(expr1a, expr2) || equals(expr1b, expr2)) {
            // (a ∧ b) ∨ a = a
            return expr2;
        }
    } else if (And * and2 = dyn_cast<And>(expr2)) {
        if (equals(and2->getOperand(0), expr1) || equals(and2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    return MAKE_NAMED_BINARY(Or, prefix, expr1, expr2);
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 == expr2) {
        return createZeroes(expr1->getType());
    } else if (isa<Ones>(expr1)) {
        return createNot(expr2);
    } else if (isa<Zeroes>(expr1)){
        return expr2;
    } else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    } else if (isa<Zeroes>(expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getOperand(0), not2->getOperand(0));
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    return MAKE_BINARY(Xor, expr1, expr2);
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix) {
    if (expr1 == expr2) {
        return createZeroes(expr1->getType());
    } else if (isa<Ones>(expr1)) {
        return createNot(expr2);
    } else if (isa<Zeroes>(expr1)){
        return expr2;
    } else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    } else if (isa<Zeroes>(expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getOperand(0), not2->getOperand(0), prefix);
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    return MAKE_NAMED_BINARY(Xor, prefix, expr1, expr2);
}

PabloAST * PabloBuilder::createAdd(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() + cast<Integer>(expr2)->value());
    } else if (isa<Integer>(expr1)) {
        if (cast<Integer>(expr1)->value() == 0) {
            return expr2;
        }
    } else if (isa<Integer>(expr2)) {
        if (cast<Integer>(expr2)->value() == 0) {
            return expr1;
        }
    }
    return MAKE_BINARY(Add, expr1, expr2);
}

PabloAST * PabloBuilder::createSubtract(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() - cast<Integer>(expr2)->value());
    } else if (isa<Integer>(expr1)) {
        if (cast<Integer>(expr1)->value() == 0) {
            return expr2;
        }
    } else if (isa<Integer>(expr2)) {
        if (cast<Integer>(expr2)->value() == 0) {
            return expr1;
        }
    }
    return MAKE_BINARY(Subtract, expr1, expr2);
}

PabloAST * PabloBuilder::createLessThan(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() < cast<Integer>(expr2)->value() ? 1 : 0);
    }
    return MAKE_BINARY(LessThan, expr1, expr2);
}

PabloAST * PabloBuilder::createEquals(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() == cast<Integer>(expr2)->value() ? 1 : 0);
    }
    return MAKE_BINARY(Equals, expr1, expr2);
}

PabloAST * PabloBuilder::createInFile(PabloAST * expr) {
    if (LLVM_UNLIKELY(isa<InFile>(expr) || isa<Zeroes>(expr))) {
        return expr;
    }
    return MAKE_UNARY(InFile, expr);
}

PabloAST * PabloBuilder::createInFile(PabloAST * expr, const llvm::StringRef prefix) {
    if (LLVM_UNLIKELY(isa<InFile>(expr) || isa<Zeroes>(expr))) {
        return expr;
    }
    return MAKE_NAMED_UNARY(InFile, prefix, expr);
}

PabloAST * PabloBuilder::createAtEOF(PabloAST * expr) {
    return MAKE_UNARY(AtEOF, expr);
}

PabloAST * PabloBuilder::createAtEOF(PabloAST * expr, const llvm::StringRef prefix) {
    return MAKE_NAMED_UNARY(AtEOF, prefix, expr);
}

PabloAST * PabloBuilder::createTerminateAt(PabloAST * strm, not_null<Integer *> code) {
    if (isa<Zeroes>(strm)) return strm;
    return MAKE_BINARY(TerminateAt, strm, code.get());
}

PabloAST * PabloBuilder::createTerminateAt(PabloAST * strm, not_null<Integer *> code, const llvm::StringRef prefix) {
    if (isa<Zeroes>(strm)) return strm;
    return MAKE_NAMED_BINARY(TerminateAt, prefix, strm, code.get());
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return MAKE_BINARY(MatchStar, marker, charclass);
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass, const llvm::StringRef prefix) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return MAKE_NAMED_BINARY(MatchStar, prefix, marker, charclass);
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return MAKE_BINARY(ScanThru, from, thru);
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef prefix) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return MAKE_NAMED_BINARY(ScanThru, prefix, from, thru);
}

PabloAST * PabloBuilder::createScanTo(PabloAST * from, PabloAST * to) {
    if (isa<Zeroes>(from) || isa<Ones>(to)) {
        return from;
    }
    return MAKE_BINARY(ScanTo, from, to);
}

PabloAST * PabloBuilder::createScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef prefix) {
    if (isa<Zeroes>(from) || isa<Ones>(to)) {
        return from;
    }
    return MAKE_NAMED_BINARY(ScanTo, prefix, from, to);
}

PabloAST * PabloBuilder::createAdvanceThenScanThru(PabloAST * from, PabloAST * thru) {
    if (isa<Zeroes>(from)) {
        return from;
    }
    if (isa<Zeroes>(thru)) {
        return createAdvance(from, getInteger(1));
    }
    return MAKE_BINARY(AdvanceThenScanThru, from, thru);
}

PabloAST * PabloBuilder::createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef prefix) {
    if (isa<Zeroes>(from)) {
        return from;
    }
    if (isa<Zeroes>(thru)) {
        return createAdvance(from, getInteger(1), prefix);
    }
    return MAKE_NAMED_BINARY(AdvanceThenScanThru, prefix, from, thru);
}

PabloAST * PabloBuilder::createAdvanceThenScanTo(PabloAST * from, PabloAST * to) {
    if (isa<Zeroes>(from)) {
        return from;
    }
    return MAKE_BINARY(AdvanceThenScanTo, from, to);
}

PabloAST * PabloBuilder::createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef prefix) {
    if (isa<Zeroes>(from)) {
        return from;
    }
    return MAKE_NAMED_BINARY(AdvanceThenScanTo, prefix, from, to);
}

PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    if (isa<Ones>(condition)) {
        return trueExpr;
    } else if (isa<Zeroes>(condition)){
        return falseExpr;
    } else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    } else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    } else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    } else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    } else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    } else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getOperand(0), falseExpr)) {
        return createXor(condition, falseExpr);
    } else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getOperand(0))){
        return createXor(condition, trueExpr);
    }
    return MAKE_TERNARY(Sel, condition, trueExpr, falseExpr);
}

PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const llvm::StringRef prefix) {
    if (isa<Ones>(condition)) {
        return trueExpr;
    } else if (isa<Zeroes>(condition)){
        return falseExpr;
    } else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    } else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    } else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    } else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    } else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    } else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getOperand(0), falseExpr)) {
        return createXor(condition, falseExpr);
    } else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getOperand(0))){
        return createXor(condition, trueExpr);
    }
    return MAKE_NAMED_TERNARY(Sel, prefix, condition, trueExpr, falseExpr);
}

PabloAST * PabloBuilder::createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3) {
    if (isa<Zeroes>(expr1) || isa<Zeroes>(expr2) || isa<Zeroes>(expr3)) {
        return createZeroes(expr1->getType());
    } else if (isa<Ones>(expr1) && isa<Ones>(expr2)) {
        return expr3;
    } else if (isa<Ones>(expr2) && isa<Ones>(expr3)) {
        return expr1;
    } else if (isa<Ones>(expr1) && isa<Ones>(expr3)) {
        return expr2;
    } else if (isa<Ones>(expr1)) {
        return createAnd(expr2, expr3);
    } else if (isa<Ones>(expr2)) {
        return createAnd(expr1, expr3);
    } else if (isa<Ones>(expr3)) {
        return createAnd(expr1, expr2);
    }
    //     (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // and3(a, b, c) =    1      0      0      0      0      0      0      0    = 0x80
    return createTernary(0x80, expr1, expr2, expr3);
}

PabloAST * PabloBuilder::createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
    if (isa<Zeroes>(expr1) || isa<Zeroes>(expr2) || isa<Zeroes>(expr3)) {
        return createZeroes(expr1->getType());
    } else if (isa<Ones>(expr1) && isa<Ones>(expr2)) {
        return expr3;
    } else if (isa<Ones>(expr2) && isa<Ones>(expr3)) {
        return expr1;
    } else if (isa<Ones>(expr1) && isa<Ones>(expr3)) {
        return expr2;
    } else if (isa<Ones>(expr1)) {
        return createAnd(expr2, expr3);
    } else if (isa<Ones>(expr2)) {
        return createAnd(expr1, expr3);
    } else if (isa<Ones>(expr3)) {
        return createAnd(expr1, expr2);
    }
    return createTernary(0x80, expr1, expr2, expr3, prefix);
}

PabloAST * PabloBuilder::createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3) {
    if (isa<Ones>(expr1) || isa<Ones>(expr2) || isa<Ones>(expr3)) {
        return createOnes(expr1->getType());
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr2)) {
        return expr3;
    } else if (isa<Zeroes>(expr2) && isa<Zeroes>(expr3)) {
        return expr1;
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr3)) {
        return expr2;
    } else if (isa<Zeroes>(expr1)) {
        return createOr(expr2, expr3);
    } else if (isa<Zeroes>(expr2)) {
        return createOr(expr1, expr3);
    } else if (isa<Zeroes>(expr3)) {
        return createOr(expr1, expr2);
    }
    //    (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // or3(a, b, c) =    1      1      1      1      1      1      1      0    = 0xFE
    return createTernary(0xFE, expr1, expr2, expr3);
}

PabloAST * PabloBuilder::createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
    if (isa<Ones>(expr1) || isa<Ones>(expr2) || isa<Ones>(expr3)) {
        return createOnes(expr1->getType());
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr2)) {
        return expr3;
    } else if (isa<Zeroes>(expr2) && isa<Zeroes>(expr3)) {
        return expr1;
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr3)) {
        return expr2;
    } else if (isa<Zeroes>(expr1)) {
        return createOr(expr2, expr3);
    } else if (isa<Zeroes>(expr2)) {
        return createOr(expr1, expr3);
    } else if (isa<Zeroes>(expr3)) {
        return createOr(expr1, expr2);
    }
    return createTernary(0xFE, expr1, expr2, expr3, prefix);
}

PabloAST * PabloBuilder::createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3) {
    if (isa<Ones>(expr1) && isa<Ones>(expr2) && isa<Ones>(expr3)) {
        return createOnes(expr1->getType());
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr2) && isa<Zeroes>(expr3)) {
        return createZeroes(expr1->getType());
    } else if (equals(expr1, expr2) && equals(expr2, expr3)) {
        return expr1;
    } else if (equals(expr1, expr2)) {
        return createXor(expr3, createZeroes(expr1->getType()));
    } else if (equals(expr1, expr3)) {
        return createXor(expr2, createZeroes(expr1->getType()));
    } else if (equals(expr2, expr3)) {
        return createXor(expr1, createZeroes(expr2->getType()));
    }
    //     (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // xor3(a, b, c) =    1      0      0      1      0      1      1      0    = 0x96
    return createTernary(0x96, expr1, expr2, expr3);
}

PabloAST * PabloBuilder::createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
    if (isa<Ones>(expr1) && isa<Ones>(expr2) && isa<Ones>(expr3)) {
        return createOnes(expr1->getType());
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr2) && isa<Zeroes>(expr3)) {
        return createZeroes(expr1->getType());
    } else if (equals(expr1, expr2) && equals(expr2, expr3)) {
        return expr1;
    } else if (equals(expr1, expr2)) {
        return createXor(expr3, createZeroes(expr1->getType()));
    } else if (equals(expr1, expr3)) {
        return createXor(expr2, createZeroes(expr1->getType()));
    } else if (equals(expr2, expr3)) {
        return createXor(expr1, createZeroes(expr2->getType()));
    }
    return createTernary(0x96, expr1, expr2, expr3, prefix);
}

PabloAST * PabloBuilder::createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3) {
    if (isa<Zeroes>(expr1) && isa<Zeroes>(expr2)) {
        return createZeroes(expr1->getType());
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr3)) {
        return createZeroes(expr1->getType());
    } else if (isa<Zeroes>(expr2) && isa<Zeroes>(expr3)) {
        return createZeroes(expr2->getType());
    } else if (isa<Ones>(expr1) && isa<Ones>(expr2)) {
        return createOnes(expr1->getType());
    } else if (isa<Ones>(expr1) && isa<Ones>(expr3)) {
        return createOnes(expr1->getType());
    } else if (isa<Ones>(expr2) && isa<Ones>(expr3)) {
        return createOnes(expr2->getType());
    }
    //          (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // majority3(a, b, c) =    1      1      1      0      1      0      0      0    = 0xE8
    return createTernary(0xE8, expr1, expr2, expr3);
}

PabloAST * PabloBuilder::createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
    if (isa<Zeroes>(expr1) && isa<Zeroes>(expr2)) {
        return createZeroes(expr1->getType());
    } else if (isa<Zeroes>(expr1) && isa<Zeroes>(expr3)) {
        return createZeroes(expr1->getType());
    } else if (isa<Zeroes>(expr2) && isa<Zeroes>(expr3)) {
        return createZeroes(expr2->getType());
    } else if (isa<Ones>(expr1) && isa<Ones>(expr2)) {
        return createOnes(expr1->getType());
    } else if (isa<Ones>(expr1) && isa<Ones>(expr3)) {
        return createOnes(expr1->getType());
    } else if (isa<Ones>(expr2) && isa<Ones>(expr3)) {
        return createOnes(expr2->getType());
    }
    return createTernary(0xE8, expr1, expr2, expr3, prefix);
}

PabloAST * PabloBuilder::createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2) {
    if (isa<Zeroes>(andExpr1)) {
        return createZeroes(andExpr1->getType());
    } else if (isa<Ones>(andExpr1)) {
        return createOr(orExpr1, orExpr2);
    } else if (isa<Ones>(orExpr1) || isa<Ones>(orExpr2)) {
        return andExpr1;
    } else if (isa<Zeroes>(orExpr1)) {
        return createAnd(andExpr1, orExpr2);
    } else if (isa<Zeroes>(orExpr2) || equals(orExpr1, orExpr2)) {
        return createAnd(andExpr1, orExpr1);
    }
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // andOr(a, b, c) =    1      1      1      0      0      0      0      0    = 0xE0
    return createTernary(0xE0, andExpr1, orExpr1, orExpr2);
}

PabloAST * PabloBuilder::createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const llvm::StringRef prefix) {
    if (isa<Zeroes>(andExpr1)) {
        return createZeroes(andExpr1->getType());
    } else if (isa<Ones>(andExpr1)) {
        return createOr(orExpr1, orExpr2);
    } else if (isa<Ones>(orExpr1) || isa<Ones>(orExpr2)) {
        return andExpr1;
    } else if (isa<Zeroes>(orExpr1)) {
        return createAnd(andExpr1, orExpr2);
    } else if (isa<Zeroes>(orExpr2) || equals(orExpr1, orExpr2)) {
        return createAnd(andExpr1, orExpr1);
    }
    return createTernary(0xE0, andExpr1, orExpr1, orExpr2, prefix);
}

PabloAST * PabloBuilder::createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2) {
    if (isa<Zeroes>(andExpr1) || equals(xorExpr1, xorExpr2)) {
        return createZeroes(andExpr1->getType());
    } else if (isa<Ones>(andExpr1)) {
        return createXor(xorExpr1, xorExpr2);
    } else if (isa<Ones>(xorExpr1) && isa<Zeroes>(xorExpr2)) {
        return andExpr1;
    } else if (isa<Zeroes>(xorExpr1) && isa<Ones>(xorExpr2)) {
        return andExpr1;
    }
    //       (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // andXor(a, b, c) =    0      1      1      0      0      0      0      0    = 0x60
    return createTernary(0x60, andExpr1, xorExpr1, xorExpr2);
}

PabloAST * PabloBuilder::createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const llvm::StringRef prefix) {
    if (isa<Zeroes>(andExpr1) || equals(xorExpr1, xorExpr2)) {
        return createZeroes(andExpr1->getType());
    } else if (isa<Ones>(andExpr1)) {
        return createXor(xorExpr1, xorExpr2);
    } else if (isa<Ones>(xorExpr1) && isa<Zeroes>(xorExpr2)) {
        return andExpr1;
    } else if (isa<Zeroes>(xorExpr1) && isa<Ones>(xorExpr2)) {
        return andExpr1;
    }
    return createTernary(0x60, andExpr1, xorExpr1, xorExpr2, prefix);
}

PabloAST * PabloBuilder::createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2) {
    if (isa<Ones>(orExpr1)) {
        return createOnes(orExpr1->getType());
    } else if (isa<Zeroes>(orExpr1)) {
        return createAnd(andExpr1, andExpr2);
    } else if (equals(andExpr1, andExpr2)) {
        return createOr(orExpr1, andExpr1);
    } else if (isa<Zeroes>(andExpr1) || isa<Zeroes>(andExpr2)) {
        return orExpr1;
    }
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // orAnd(a, b, c) =    1      1      1      1      1      0      0      0    = 0xF8
    return createTernary(0xF8, orExpr1, andExpr1, andExpr2);
}

PabloAST * PabloBuilder::createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const llvm::StringRef prefix) {
    if (isa<Ones>(orExpr1)) {
        return createOnes(orExpr1->getType());
    } else if (isa<Zeroes>(orExpr1)) {
        return createAnd(andExpr1, andExpr2);
    } else if (equals(andExpr1, andExpr2)) {
        return createOr(orExpr1, andExpr1);
    } else if (isa<Zeroes>(andExpr1) || isa<Zeroes>(andExpr2)) {
        return orExpr1;
    }
    return createTernary(0xF8, orExpr1, andExpr1, andExpr2, prefix);
}

PabloAST * PabloBuilder::createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2) {
    if (isa<Ones>(orExpr1)) {
        return createOnes(orExpr1->getType());
    } else if (isa<Zeroes>(orExpr1)) {
        return createXor(xorExpr1, xorExpr2);
    } else if (equals(xorExpr1, xorExpr2)) {
        return orExpr1;
    } else if (isa<Ones>(xorExpr1) && isa<Zeroes>(xorExpr2)) {
        return createOnes(orExpr1->getType());
    } else if (isa<Zeroes>(xorExpr1) && isa<Ones>(xorExpr2)) {
        return createOnes(orExpr1->getType());
    }
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // orXor(a, b, c) =    1      1      1      1      0      1      1      0    = 0xF6
    return createTernary(0xF6, orExpr1, xorExpr1, xorExpr2);
}

PabloAST * PabloBuilder::createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const llvm::StringRef prefix) {
    if (isa<Ones>(orExpr1)) {
        return createOnes(orExpr1->getType());
    } else if (isa<Zeroes>(orExpr1)) {
        return createXor(xorExpr1, xorExpr2);
    } else if (equals(xorExpr1, xorExpr2)) {
        return orExpr1;
    } else if (isa<Ones>(xorExpr1) && isa<Zeroes>(xorExpr2)) {
        return createOnes(orExpr1->getType());
    } else if (isa<Zeroes>(xorExpr1) && isa<Ones>(xorExpr2)) {
        return createOnes(orExpr1->getType());
    }
    return createTernary(0xF6, orExpr1, xorExpr1, xorExpr2, prefix);
}

PabloAST * PabloBuilder::createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2) {
    if (equals(andExpr1, andExpr2)) {
        return createXor(xorExpr1, andExpr1);
    } else if (isa<Zeroes>(andExpr1) || isa<Ones>(andExpr2)) {
        return createXor(xorExpr1, andExpr1);
    } else if (isa<Ones>(andExpr1) || isa<Zeroes>(andExpr2)) {
        return createXor(xorExpr1, andExpr2);
    }
    //       (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // xorAnd(a, b, c) =    0      1      1      1      1      0      0      0    = 0x78
    return createTernary(0x78, xorExpr1, andExpr1, andExpr2);
}

PabloAST * PabloBuilder::createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const llvm::StringRef prefix) {
    if (equals(andExpr1, andExpr2)) {
        return createXor(xorExpr1, andExpr1);
    } else if (isa<Zeroes>(andExpr1) || isa<Ones>(andExpr2)) {
        return createXor(xorExpr1, andExpr1);
    } else if (isa<Ones>(andExpr1) || isa<Zeroes>(andExpr2)) {
        return createXor(xorExpr1, andExpr2);
    }
    return createTernary(0x78, xorExpr1, andExpr1, andExpr2, prefix);
}

PabloAST * PabloBuilder::createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2) {
    if (isa<Ones>(orExpr1) || isa<Ones>(orExpr2)) {
        return createXor(xorExpr1, createOnes(xorExpr1->getType()));
    } else if (equals(orExpr1, orExpr1)) {
        return createXor(xorExpr1, createZeroes(xorExpr1->getType()));
    } else if (isa<Ones>(orExpr1) || isa<Zeroes>(orExpr2)) {
        return createXor(xorExpr1, orExpr1);
    } else if (isa<Zeroes>(orExpr1) || isa<Ones>(orExpr2)) {
        return createXor(xorExpr1, orExpr2);
    }
    //      (a, b, c) =  (111), (110), (101), (100), (011), (010), (001), (000)
    // xorOr(a, b, c) =    0      0      0      1      1      1      1      0    = 0x1E
    return createTernary(0x1E, xorExpr1, orExpr1, orExpr2);
}

PabloAST * PabloBuilder::createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const llvm::StringRef prefix) {
    if (isa<Ones>(orExpr1) || isa<Ones>(orExpr2)) {
        return createXor(xorExpr1, createOnes(xorExpr1->getType()));
    } else if (equals(orExpr1, orExpr1)) {
        return createXor(xorExpr1, createZeroes(xorExpr1->getType()));
    } else if (isa<Ones>(orExpr1) || isa<Zeroes>(orExpr2)) {
        return createXor(xorExpr1, orExpr1);
    } else if (isa<Zeroes>(orExpr1) || isa<Ones>(orExpr2)) {
        return createXor(xorExpr1, orExpr2);
    }
    return createTernary(0x1E, xorExpr1, orExpr1, orExpr2, prefix);
}

PabloAST * PabloBuilder::createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c) {
    assert (mask->value() <= 0xFF);
    return MAKE_QUATERNARY(Ternary, mask, a, b, c);
}

PabloAST * PabloBuilder::createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c, const llvm::StringRef prefix) {
    assert (mask->value() <= 0xFF);
    return MAKE_NAMED_QUATERNARY(Ternary, prefix, mask, a, b, c);
}

PabloAST * PabloBuilder::createIntrinsicCall(Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv) {
    struct __intrinsic_functor {
        inline PabloAST * operator () (Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv) {
            return cast<PabloAST>(mPb->createIntrinsicCall(intrinsic, argv));
        }
        inline __intrinsic_functor(PabloBlock * pb)
        : mPb(pb)
        {}
        PabloBlock * const mPb;
    };
    __intrinsic_functor functor(mPb);
    return cast<PabloAST>(mExprTable.findIntrinsicOrCall(functor, TypeId::IntrinsicCall, intrinsic, argv));
}

PabloAST * PabloBuilder::createIntrinsicCall(Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv, const llvm::StringRef prefix) {
    struct __intrinsic_functor {
        inline PabloAST * operator () (Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv) {
            return cast<PabloAST>(mPb->createIntrinsicCall(intrinsic, argv, mPrefix));
        }
        inline __intrinsic_functor(PabloBlock * pb, llvm::StringRef prefix)
        : mPb(pb), mPrefix(prefix)
        {}
        PabloBlock * const mPb;
        Intrinsic          mIn;
        llvm::StringRef    mPrefix;
    };
    __intrinsic_functor functor(mPb, prefix);
    return cast<PabloAST>(mExprTable.findIntrinsicOrCall(functor, TypeId::IntrinsicCall, intrinsic, argv));
}



}
