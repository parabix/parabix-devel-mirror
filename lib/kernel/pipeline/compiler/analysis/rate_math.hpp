#ifndef RATE_MATH_HPP
#define RATE_MATH_HPP

#include "../pipeline_compiler.hpp"

#define BEGIN_SCOPED_REGION {
#define END_SCOPED_REGION }

namespace kernel {

class Expr {
protected:
    enum class TypeId : unsigned {
        Constant
        , Variable
        , Pair
        , Add
        , Subtract
        , Multiply
        , Min
        , Max
    };

    TypeId getTypeId() const noexcept {
        return mTypeId;
    }

    #ifdef HAS_Z3
    using ObjectType = Z3_ast;
    #else
    using ObjectType = void *;
    #endif

    Expr(const TypeId typeId, ObjectType & obj)
    : mTypeId(typeId)
    , mObject(obj) {

    }

    ObjectType getObject() const {
        return mObject;
    }

public:

    static inline bool classof(const Expr *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

protected:
    const TypeId    mTypeId;
    ObjectType      mObject;
};

#define CLASS_OF(Type) \
    static inline bool classof(const Expr * e) { \
        return e->getTypeId() == TypeId::Type; \
    }

class Constant : public Expr {
public:
    CLASS_OF(Constant)

    const Rational & getValue() const {
        return mValue;
    }

    Constant(const Rational & value, ObjectType & obj)
    : Expr(TypeId::Constant, obj)
    , mValue(value) {

    }

private:
    Rational mValue;
};


class Variable : public Expr {
public:
    CLASS_OF(Variable)

    Variable(ObjectType & obj)
    : Expr(TypeId::Variable, obj) {

    }

};


class Pair : public Expr {
public:

    static inline bool classof(const Expr * e) { \
        return e->getTypeId() > TypeId::Pair; \
    }

    const Expr * getLeft() const {
        return mLeft;
    }

    const Expr * getRight() const {
        return mRight;
    }

protected:

    Pair(const TypeId typeId, const Expr * left, const Expr * right, ObjectType & obj)
    : Expr(typeId, obj)
    , mLeft(left)
    , mRight(right) {

    }


private:
    const Expr * const mLeft;
    const Expr * const mRight;
};

#define MAKE_PAIR(Type) \
class Type final : public Pair { \
public: \
    CLASS_OF(Type) \
    Type(const Expr * left, const Expr * right, ObjectType obj) \
    : Pair(TypeId::Type, left, right, obj)  { \
    } \
};

MAKE_PAIR(Add)
MAKE_PAIR(Subtract)
MAKE_PAIR(Multiply)
MAKE_PAIR(Min)
MAKE_PAIR(Max)

#undef MAKE_PAIR
#undef CLASS_OF

/** ------------------------------------------------------------------------------------------------------------- *
 * constant
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::constant(const Rational value) {
    #ifdef HAS_Z3
    auto val = Z3_mk_real(mZ3Context, value.numerator(), value.denominator());
    #else
    auto val = nullptr;
    #endif
    return new (mAllocator) Constant(value, val);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * variable
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::variable() {
    #ifdef HAS_Z3
    auto var = Z3_mk_fresh_const(mZ3Context, nullptr, Z3_mk_real_sort(mZ3Context));
    // we only care about non-negative values
    auto c1 = Z3_mk_ge(mZ3Context, var, Z3_mk_real(mZ3Context, 0, 1));
    Z3_solver_assert(mZ3Context, mZ3Solver, c1);
    #else
    auto var = nullptr;
    #endif
    return new (mAllocator) Variable(var);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * add
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::add(const Expr * X, const Expr * Y) {
    if (isa<Constant>(X) && isa<Constant>(Y)) {
        return constant(cast<Constant>(X)->getValue() + cast<Constant>(Y)->getValue());
    }

    if (isa<Add>(Y)) {
        std::swap(X, Y);
    }

    // (A + B) + (C + D) = (A + C) + (B + D)
    // (A + B) + (C - D) = (A + C) + (B - D)
    // (A - B) + (C - D) = (A + C) - (B + D)
    if (isa<Add>(X) || isa<Subtract>(X)) {
        if (isa<Add>(Y) || isa<Subtract>(Y)) {
            const Expr * const A = cast<Pair>(X->getLeft());
            const Expr * const C = cast<Pair>(Y->getLeft());
            if (isa<Constant>(A) && isa<Constant>(C)) {
                const Expr * const B = cast<Pair>(X->getRight());
                const Expr * const D = cast<Pair>(Y->getRight());
                const Expr * const E = add(A, C);
                assert(isa<Constant>(E));
                const Expr * F = nullptr;
                if (X->getTypeId() == Y->getTypeId()) {
                    F = add(B, D);
                } else {
                    F = subtract(B, D);
                }
                if (isa<Add>(X)) {
                    X = E;
                    Y = F;
                } else {
                    return subtract(E, F);
                }

            }
        }
    }

    if (isa<Constant>(Y)) {
        std::swap(X, Y);
    }

    // X + (C + D)
    // X + (C - D)

    if (isa<Constant>(X)) {
        if (isa<Add>(Y) || isa<Subtract>(Y)) {
            const Expr * const C = cast<Pair>(Y->getLeft());
            if (isa<Constant>(C)) {
                X = add(X, C, alloc); assert(isa<Constant>(X));
                Y = cast<Pair>(Y)->getRight();
                if (isa<Add>(Y)) {
                    return add(X, Y, alloc);
                } else {
                    return subtract(X, Y, alloc);
                }
            }
        }
    }

    if (isa<Min>(X) || isa<Max>(X)) {
        std::swap(X, Y);
    }

    // X + min(C, D) = min(X+C, X+D)
    // X + max(C, D) = max(X+C, X+D)

    if ((isa<Min>(Y) || isa<Max>(Y)) && (!isa<Min>(X) && !isa<Max>(X))) {
        const Expr * const C = cast<Pair>(Y->getLeft());
        const Expr * const D = cast<Pair>(Y->getRight());
        const Expr * const E = add(X, C);
        const Expr * const F = add(X, D);
        if (isa<Min>(Y)) {
            return min(E, F);
        } else {
            return max(E, F);
        }
    }

    #ifdef HAS_Z3
    Z3_ast args[2] = { X->getObject(), Y->getObject() };
    auto result = Z3_mk_add(mZ3Context, 2, args);
    #else
    auto result = nullptr;
    #endif
    return new (mAllocator) Add(X, Y, result);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * subtract
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::subtract(const Expr * X, const Expr * Y) {
    if (isa<Constant>(X) && isa<Constant>(Y)) {
        return constant(cast<Constant>(X)->getValue() - cast<Constant>(Y)->getValue());
    }

    // (A - B) - (C + D) = (A - C) - (B + D)
    // (A + B) - (C + D) = (A - C) - (D - B)

    // (A + B) - (C - D) = (A - C) + (B + D)
    // (A - B) - (C - D) = (A - C) + (D - B)

    if (isa<Add>(X) || isa<Subtract>(X)) {

        if (isa<Add>(Y) || isa<Subtract>(Y)) {
            const Expr * const A = cast<Pair>(X->getLeft());
            const Expr * const C = cast<Pair>(Y->getLeft());
            if (isa<Constant>(A) && isa<Constant>(C)) {
                const Expr * const B = cast<Pair>(X->getRight());
                const Expr * const D = cast<Pair>(Y->getRight());
                const Expr * const Y = subtract(A, C);
                assert (isa<Constant>(Y));
                const Expr * Z = nullptr;
                if (X->getTypeId() == Y->getTypeId()) {
                    Z = subtract(D, B);
                } else {
                    Z = add(B, D);
                }
                if (isa<Add>(Y)) {
                    return subtract(Y, Z);
                } else {
                    return add(Y, Z);
                }
            }
        }
    }


    bool negated = false;
    if (isa<Constant>(Y)) {
        std::swap(X, Y);
        negated = true;
    }

    // X - (C + D) = (X - C) - D
    // X - (C - D) = (X - C) + D

    if (isa<Constant>(X)) {
        if (isa<Add>(Y) || isa<Subtract>(Y)) {
            const Expr * C = cast<Pair>(Y->getLeft());
            if (isa<Constant>(C)) {

                // -((X - C) - D) = (C - X) + D
                // -((X - C) + D) = (C - X) - D

                if (negated) {
                    std::swap(X, C);
                }
                X = subtract(X, C); assert(isa<Constant>(X));
                Y = cast<Pair>(Y)->getRight();
                if (isa<Add>(Y) ^ negated) {
                    return add(X, Y);
                }
            }
        }
    }

    if (isa<Min>(X) || isa<Max>(X)) {
        std::swap(X, Y);
        negated ^= true;
    }

    // X - min(C, D) = min(X-C, X-D)
    // X - max(C, D) = max(X-C, X-D)

    if ((isa<Min>(Y) || isa<Max>(Y)) && (!isa<Min>(X) && !isa<Max>(X))) {
        const Expr * C = cast<Pair>(Y->getLeft());
        const Expr * D = cast<Pair>(Y->getRight());
        const Expr * X1 = X;
        const Expr * X2 = X;
        if (negated) {
            std::swap(X1, C);
            std::swap(X2, D);
        }
        const Expr * const E = subtract(C, X1);
        const Expr * const F = subtract(D, X2);
        if (isa<Min>(Y)) {
            return min(E, F);
        } else {
            return max(E, F);
        }
    }


    if (negated) {
        std::swap(X, Y);
    }

    #ifdef HAS_Z3
    Z3_ast args[2] = { X->getObject(), Y->getObject() };
    auto result = Z3_mk_sub(mZ3Context, 2, args);
    #else
    auto result = nullptr;
    #endif
    return new (mAllocator) Subtract(X, Y, result);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * multiply
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::multiply(const Expr * X, const Expr * Y) {
    if (isa<Constant>(X) && isa<Constant>(Y)) {
        return constant(cast<Constant>(X)->getValue() * cast<Constant>(Y)->getValue());
    }
    if (isa<Constant>(Y)) {
        std::swap(X, Y);
    }

    // X * (C + D) = (X * C) + (X * D)
    // X * (C - D) = (X * C) - (X * D)
    // X * (C * D) = (X * C) * D

    if (isa<Constant>(X)) {
        if (isa<Add>(Y) || isa<Subtract>(Y) || isa<Multiply>(Y)) {
            const Expr * const C = cast<Pair>(Y->getLeft());
            if (isa<Constant>(C)) {
                const Expr * const D = cast<Pair>(Y->getRight());
                const Expr * const E = multiply(X, C);
                if (isa<Multiply>(Y)) {
                    X = E;
                    Y = D;
                } else {
                    const Expr * const F = multiply(X, D);
                    if (isa<Add>(Y)) {
                        return add(E, F);
                    } else {
                        return subtract(E, F);
                    }
                }
            }
        }
    }

    if (isa<Min>(X) || isa<Max>(X)) {
        std::swap(X, Y);
    }

    // X * min(C, D) = min(X*C, X*D)
    // X * max(C, D) = max(X*C, X*D)

    if ((isa<Min>(Y) || isa<Max>(Y)) && (!isa<Min>(X) && !isa<Max>(X))) {
        const Expr * const C = cast<Pair>(Y->getLeft());
        const Expr * const D = cast<Pair>(Y->getRight());
        const Expr * const E = multiply(X, C);
        const Expr * const F = multiply(X, D);
        if (isa<Min>(Y)) {
            return min(E, F);
        } else {
            return max(E, F);
        }
    }

    #ifdef HAS_Z3
    Z3_ast args[2] = { X->getObject(), Y->getObject() };
    auto result = Z3_mk_mul(mZ3Context, 2, args);
    #else
    auto result = nullptr;
    #endif
    return new (mAllocator) Multiply(X, Y, result);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * min
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::min(const Expr * X, const Expr * Y) {
    if (isa<Constant>(X) && isa<Constant>(Y)) {
        if (cast<Constant>(X)->getValue() < cast<Constant>(Y)->getValue()) {
            return X;
        } else {
            return Y;
        }
    }

    #ifdef HAS_Z3
    auto lessThan = [&](Z3_ast const x, Z3_ast const y) {
        Z3_solver_push(mZ3Context, mZ3Solver);
        Z3_solver_assert(mZ3Context, mZ3Solver, Z3_mk_lt(mZ3Context, x, y));
        const auto result = Z3_solver_check(mZ3Context, mZ3Solver);
        Z3_solver_pop(mZ3Context, mZ3Solver, 1);
        return result;
    };

    auto const x = X->getObject();
    auto const y = Y->getObject();

    const auto exists_X_lt_Y = lessThan(x, y);
    if (LLVM_LIKELY(exists_X_lt_Y != Z3_L_UNDEF)) {
        const auto exists_Y_lt_X = lessThan(y, x);
        if (LLVM_LIKELY(exists_Y_lt_X != Z3_L_UNDEF)) {
            if (exists_X_lt_Y == Z3_L_TRUE && exists_Y_lt_X == Z3_L_FALSE) {
                return X;
            }
            if (exists_Y_lt_X == Z3_L_TRUE && exists_X_lt_Y == Z3_L_FALSE) {
                return Y;
            }
        }
    }

    // undefined or both have satisfying assignments
    auto q = Z3_mk_fresh_const(mZ3Context, nullptr, Z3_mk_real_sort(mZ3Context));
    auto c1 = Z3_mk_le(mZ3Context, q, x);
    Z3_solver_assert(mZ3Context, mZ3Solver, c1);
    auto c2 = Z3_mk_le(mZ3Context, q, y);
    Z3_solver_assert(mZ3Context, mZ3Solver, c2);
    auto c3 = Z3_mk_ge(mZ3Context, var, Z3_mk_real(mZ3Context, 0, 1));
    Z3_solver_assert(mZ3Context, mZ3Solver, c3);
    #else
    auto q = nullptr;
    #endif

    return new (mAllocator) Min(X, Y, q);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * max
 ** ------------------------------------------------------------------------------------------------------------- */
Expr * PipelineCompiler::max(const Expr * X, const Expr * Y) {
    if (isa<Constant>(X) && isa<Constant>(Y)) {
        if (cast<Constant>(X)->getValue() > cast<Constant>(Y)->getValue()) {
            return X;
        } else {
            return Y;
        }
    }

    #ifdef HAS_Z3
    auto greaterThan = [&](Z3_ast const x, Z3_ast const y) {
        Z3_solver_push(mZ3Context, mZ3Solver);
        Z3_solver_assert(mZ3Context, mZ3Solver, Z3_mk_gt(mZ3Context, x, y));
        const auto result = Z3_solver_check(mZ3Context, mZ3Solver);
        Z3_solver_pop(mZ3Context, mZ3Solver, 1);
        return result;
    };

    auto const x = X->getObject();
    auto const y = Y->getObject();

    const auto exists_X_gt_Y = greaterThan(x, y);
    if (LLVM_LIKELY(exists_X_gt_Y != Z3_L_UNDEF)) {
        const auto exists_Y_gt_X = greaterThan(y, x);
        if (LLVM_LIKELY(exists_Y_gt_X != Z3_L_UNDEF)) {
            if (exists_X_gt_Y == Z3_L_TRUE && exists_Y_gt_X == Z3_L_FALSE) {
                return X;
            }
            if (exists_Y_gt_X == Z3_L_TRUE && exists_X_gt_Y == Z3_L_FALSE) {
                return Y;
            }
        }
    }

    // undefined or both have satisfying assignments
    auto q = Z3_mk_fresh_const(mZ3Context, nullptr, Z3_mk_real_sort(mZ3Context));
    auto c1 = Z3_mk_ge(mZ3Context, q, x);
    Z3_solver_assert(mZ3Context, mZ3Solver, c1);
    auto c2 = Z3_mk_ge(mZ3Context, q, y);
    Z3_solver_assert(mZ3Context, mZ3Solver, c2);
    auto c3 = Z3_mk_ge(mZ3Context, var, Z3_mk_real(mZ3Context, 0, 1));
    Z3_solver_assert(mZ3Context, mZ3Solver, c3);
    #else
    auto q = nullptr;
    #endif

    return new (mAllocator) Max(X, Y, q);
}

} // end of namespace kernel

#endif // RATE_MATH_HPP
