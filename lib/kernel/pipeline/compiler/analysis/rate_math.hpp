#ifndef RATE_MATH_HPP
#define RATE_MATH_HPP

#include "../pipeline_compiler.hpp"

/* This file simply abstracts the Z3 solver in case an alternate solver is available and/or required later. */

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * constant
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::constant(const Rational value) const {
    if (value.denominator() == 1) {
        return Z3_mk_int(mZ3Context, value.numerator(), Z3_mk_int_sort(mZ3Context));
    } else {
        return Z3_mk_real(mZ3Context, value.numerator(), value.denominator());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * variable
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::free_variable(const ExprVarType type) const {
    Z3_sort S;
    switch (type) {
        case ExprVarType::INT:
            S = Z3_mk_int_sort(mZ3Context);
            break;
        case ExprVarType::REAL:
            S = Z3_mk_real_sort(mZ3Context);
            break;
    }
    auto v = Z3_mk_fresh_const(mZ3Context, nullptr, S);
    // we only care about non-negative values
    auto c1 = Z3_mk_ge(mZ3Context, v, mZ3_ZERO);
    Z3_solver_assert(mZ3Context, mZ3Solver, c1);
    return v;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * bounded_variable
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::bounded_variable(Expr inclusive_lb, Expr inclusive_ub) const {
    auto v = Z3_mk_fresh_const(mZ3Context, nullptr, Z3_mk_real_sort(mZ3Context));
    auto c1 = Z3_mk_ge(mZ3Context, v, inclusive_lb);
    Z3_solver_assert(mZ3Context, mZ3Solver, c1);
    auto c2 = Z3_mk_le(mZ3Context, v, inclusive_ub);
    Z3_solver_assert(mZ3Context, mZ3Solver, c2);
    return v;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * bounded_variable
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::bounded_variable(const Rational inclusive_lb, const Rational inclusive_ub) const {
    assert (inclusive_lb <= inclusive_ub);
    if (LLVM_UNLIKELY(inclusive_lb == inclusive_ub)) {
        return constant(inclusive_lb);
    }
    auto lb = Z3_mk_real(mZ3Context, inclusive_lb.numerator(), inclusive_lb.denominator());
    auto ub = Z3_mk_real(mZ3Context, inclusive_ub.numerator(), inclusive_ub.denominator());
    return bounded_variable(lb, ub);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * add
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::add(Expr X, Expr Y) const {
    Z3_ast args[2] = { X, Y };
    return Z3_simplify(mZ3Context, Z3_mk_add(mZ3Context, 2, args));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * subtract
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::subtract(Expr X, Expr Y) const {
    Z3_ast args[2] = { X, Y };
    return Z3_simplify(mZ3Context, Z3_mk_sub(mZ3Context, 2, args));
}


/** ------------------------------------------------------------------------------------------------------------- *
 * multiply
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::multiply(Expr X, Expr Y) const {
    Z3_ast args[2] = { X, Y };
    return Z3_simplify(mZ3Context, Z3_mk_mul(mZ3Context, 2, args));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * divide
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::divide(Expr X, Expr Y) const {
    return Z3_simplify(mZ3Context, Z3_mk_div(mZ3Context, X, Y));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * equals
 ** ------------------------------------------------------------------------------------------------------------- */
inline Expr PipelineCompiler::equals(Expr X, Expr Y) const {
    return Z3_simplify(mZ3Context, Z3_mk_eq(mZ3Context, X, Y));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * min
 ** ------------------------------------------------------------------------------------------------------------- */
Expr PipelineCompiler::mk_min(const ExprSet & set) const {

    // check whether we can statically prove which values are always minimal

    const auto n = set.size();

    if (LLVM_UNLIKELY(n == 1)) {
        return *set.begin();
    }

    BitVector filtered(n);
    for (auto i = 1U; i != n; ++i) {

        if (filtered.test(i)) {
            continue;
        }

        auto filter = [&](const unsigned begin, const unsigned end) {
            for (auto j = begin; j != end; ++j) {

                if (filtered.test(j)) {
                    continue;
                }

                // ∄x : f_j(x) < f_i(x)
                Z3_solver_push(mZ3Context, mZ3Solver);
                const auto c = Z3_mk_lt(mZ3Context, *set.nth(j), *set.nth(i));
                Z3_solver_assert(mZ3Context, mZ3Solver, c);
                const auto result = Z3_solver_check(mZ3Context, mZ3Solver);
                Z3_solver_pop(mZ3Context, mZ3Solver, 1);

                if (result == Z3_L_FALSE) {
                    filtered.set(j);
                }
            }
        };

        filter(0U, i);
        filter(i + 1U, n);
    }

    const auto m = filtered.count();
    if (LLVM_UNLIKELY(m == (n - 1U))) {
        const auto i = filtered.find_first_unset();
        assert (i != -1);
        return *set.nth(i);
    }

    // at least two have minimal assignments; create a new variable with the appropriate constraints
    auto q = free_variable();
    for (auto i = 0U; i != n; ++i) {
        if (filtered.test(i)) {
            continue;
        }
        auto c = Z3_mk_le(mZ3Context, q, *set.nth(i));
        Z3_solver_assert(mZ3Context, mZ3Solver, c);
    }
    return q;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * max
 ** ------------------------------------------------------------------------------------------------------------- */
Expr PipelineCompiler::mk_max(const ExprSet & set) const {
    // check whether we can statically prove which values are always minimal

    const auto n = set.size();

    if (LLVM_UNLIKELY(n == 1)) {
        return *set.begin();
    }

    BitVector filtered(n);
    for (auto i = 1U; i != n; ++i) {

        if (filtered.test(i)) {
            continue;
        }

        auto filter = [&](const unsigned begin, const unsigned end) {
            for (auto j = begin; j != end; ++j) {

                if (filtered.test(j)) {
                    continue;
                }

                // ∄x : f_j(x) > f_i(x)
                Z3_solver_push(mZ3Context, mZ3Solver);
                const auto c = Z3_mk_gt(mZ3Context, *set.nth(j), *set.nth(i));
                Z3_solver_assert(mZ3Context, mZ3Solver, c);
                const auto result = Z3_solver_check(mZ3Context, mZ3Solver);
                Z3_solver_pop(mZ3Context, mZ3Solver, 1);

                if (result == Z3_L_FALSE) {
                    filtered.set(j);
                }
            }
        };

        filter(0U, i);
        filter(i + 1U, n);
    }

    const auto m = filtered.count();
    if (LLVM_UNLIKELY(m == (n - 1U))) {
        const auto i = filtered.find_first_unset();
        assert (i != -1);
        return *set.nth(i);
    }

    // at least two have maximal assignments; create a new variable with the appropriate constraints
    auto q = free_variable();
    for (auto i = 0U; i != n; ++i) {
        if (filtered.test(i)) {
            continue;
        }
        auto c = Z3_mk_ge(mZ3Context, q, *set.nth(i));
        Z3_solver_assert(mZ3Context, mZ3Solver, c);
    }
    return q;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * mk_floor
 ** ------------------------------------------------------------------------------------------------------------- */
Expr PipelineCompiler::mk_floor(Expr X) const {
    // Note: since there is no floor function nor do I see a way of obtaining the num/denom of X,
    // I'm relying on the fact:

    // ⌊X⌋ = X - (X % 1)

    auto R = Z3_mk_rem(mZ3Context, X, mZ3_ONE);
    Z3_ast args[2] = { X, R };
    return Z3_simplify(mZ3Context, Z3_mk_sub(mZ3Context, 2, args));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * mk_ceiling
 ** ------------------------------------------------------------------------------------------------------------- */
Expr PipelineCompiler::mk_ceiling(Expr X) const {
    // Note: since there is no ceiling function nor do I see a way of obtaining the num/denom of X,
    // I'm relying on the fact:

    // ⌈X⌉ = −⌊−X⌋

    Z3_ast args[2] = { mZ3_ZERO, X };
    args[1] = mk_floor(Z3_mk_sub(mZ3Context, 2, args));
    return Z3_simplify(mZ3Context, Z3_mk_sub(mZ3Context, 2, args));

}

/** ------------------------------------------------------------------------------------------------------------- *
 * addConstraint
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addConstraint(Expr C) const {
    Z3_solver_assert(mZ3Context, mZ3Solver, C);
}


#if 0
/** ------------------------------------------------------------------------------------------------------------- *
 * isConstant
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isConstant(Expr X) const {
    const Z3_ast_kind r = Z3_get_ast_kind(mZ3Context, X);
    if (r == Z3_NUMERAL_AST && r == Z3_APP_AST) {
        return Z3_get_app_num_args(mZ3Context, X) == 0;
    }
    return false;
}
#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * start_smt_solver
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::start_smt_solver() {
    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "false");
    mZ3Context = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    mZ3Solver = Z3_mk_solver(mZ3Context);
    Z3_solver_inc_ref(mZ3Context, mZ3Solver);
    mZ3_ZERO = Z3_mk_real(mZ3Context, 0, 1);
    mZ3_ONE = Z3_mk_real(mZ3Context, 1, 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * destroy_smt_solver
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::destroy_smt_solver() {
    mZ3_ONE = nullptr;
    mZ3_ZERO = nullptr;
    Z3_solver_dec_ref(mZ3Context, mZ3Solver);
    mZ3Solver = nullptr;
    Z3_del_context(mZ3Context);
    mZ3Context = nullptr;
}


} // end of namespace kernel

#endif // RATE_MATH_HPP
