#ifndef MAXSAT_HPP
#define MAXSAT_HPP

#include <llvm/Support/ErrorHandling.h>
#include <algorithm>
#include <vector>
#include <z3.h>


inline Z3_ast mk_binary_or(Z3_context ctx, Z3_ast in_1, Z3_ast in_2) {
    Z3_ast args[2] = { in_1, in_2 };
    return Z3_mk_or(ctx, 2, args);
}

inline Z3_ast mk_ternary_or(Z3_context ctx, Z3_ast in_1, Z3_ast in_2, Z3_ast in_3) {
    Z3_ast args[3] = { in_1, in_2, in_3 };
    return Z3_mk_or(ctx, 3, args);
}

inline Z3_ast mk_binary_and(Z3_context ctx, Z3_ast in_1, Z3_ast in_2) {
    Z3_ast args[2] = { in_1, in_2 };
    return Z3_mk_and(ctx, 2, args);
}

/**
   \brief Create an adder for inputs of size \c num_bits.
   The arguments \c in1 and \c in2 are arrays of bits of size \c num_bits.

   \remark \c result must be an array of size \c num_bits + 1.
*/
inline void mk_adder(Z3_context ctx, const unsigned num_bits, Z3_ast * in_1, Z3_ast * in_2, Z3_ast * result) {
    Z3_ast cin = Z3_mk_false(ctx);
    for (unsigned i = 0; i < num_bits; i++) {
        result[i] = Z3_mk_xor(ctx, Z3_mk_xor(ctx, in_1[i], in_2[i]), cin);
        cin = mk_ternary_or(ctx, mk_binary_and(ctx, in_1[i], in_2[i]), mk_binary_and(ctx, in_1[i], cin), mk_binary_and(ctx, in_2[i], cin));
    }
    result[num_bits] = cin;
}

/**
   \brief Given \c num_ins "numbers" of size \c num_bits stored in \c in.
   Create floor(num_ins/2) adder circuits. Each circuit is adding two consecutive "numbers".
   The numbers are stored one after the next in the array \c in.
   That is, the array \c in has size num_bits * num_ins.
   Return an array of bits containing \c ceil(num_ins/2) numbers of size \c (num_bits + 1).
   If num_ins/2 is not an integer, then the last "number" in the output, is the last "number" in \c in with an appended "zero".
*/
inline unsigned mk_adder_pairs(Z3_context ctx, const unsigned num_bits, const unsigned num_ins, Z3_ast * in, Z3_ast * out) {
    unsigned out_num_bits = num_bits + 1;
    Z3_ast * _in          = in;
    Z3_ast * _out         = out;
    unsigned out_num_ins  = (num_ins % 2 == 0) ? (num_ins / 2) : (num_ins / 2) + 1;
    for (unsigned i = 0; i < num_ins / 2; i++) {
        mk_adder(ctx, num_bits, _in, _in + num_bits, _out);
        _in  += num_bits;
        _in  += num_bits;
        _out += out_num_bits;
    }
    if (num_ins % 2 != 0) {
        for (unsigned i = 0; i < num_bits; i++) {
            _out[i] = _in[i];
        }
        _out[num_bits] = Z3_mk_false(ctx);
    }
    return out_num_ins;
}

/**
   \brief Return the \c idx bit of \c val.
*/
inline bool get_bit(unsigned val, unsigned idx) {
    return (val & (1U << (idx & 31))) != 0;
}

/**
   \brief Given an integer val encoded in n bits (boolean variables), assert the constraint that val <= k.
*/
inline void assert_le_one(Z3_context ctx, Z3_solver s, unsigned n, Z3_ast * val) {
    Z3_ast i1, i2;
    Z3_ast not_val = Z3_mk_not(ctx, val[0]);
    assert (get_bit(1, 0));
    Z3_ast out = Z3_mk_true(ctx);
    for (unsigned i = 1; i < n; i++) {
        not_val = Z3_mk_not(ctx, val[i]);
        if (get_bit(1, i)) {
            i1 = not_val;
            i2 = out;
        } else {
            i1 = Z3_mk_false(ctx);
            i2 = Z3_mk_false(ctx);
        }
        out = mk_ternary_or(ctx, i1, i2, mk_binary_and(ctx, not_val, out));
    }
    // Z3_mk_atmost ?
    Z3_solver_assert(ctx, s, out);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * Fu & Malik procedure for MaxSAT. This procedure is based on unsat core extraction and the at-most-one constraint.
 ** ------------------------------------------------------------------------------------------------------------- */
static int Z3_maxsat(Z3_context ctx, Z3_solver solver, std::vector<Z3_ast> soft) {
    assert("initial formula is unsatisfiable!" && (Z3_solver_check(ctx, solver) != Z3_L_FALSE));
    if (LLVM_UNLIKELY(soft.empty())) {
        return 0;
    }
    const auto n = soft.size();
    const auto ty = Z3_mk_bool_sort(ctx);

    std::vector<Z3_ast> aux_vars(n);
    std::vector<Z3_ast> assumptions(n);

    for (unsigned i = 0; i < n; ++i) {
        aux_vars[i] = Z3_mk_fresh_const(ctx, nullptr, ty);
        Z3_solver_assert(ctx, solver, mk_binary_or(ctx, soft[i], aux_vars[i]));
    }

    for (auto c = n; c; --c) {
        // create assumptions
        for (unsigned i = 0; i < n; i++) {
            // Recall that we asserted (soft_cnstrs[i] \/ aux_vars[i])
            // So using (NOT aux_vars[i]) as an assumption we are force the soft constraints to be considered.
            assumptions[i] = Z3_mk_not(ctx, aux_vars[i]);
        }
        if (Z3_solver_check_assumptions(ctx, solver, n, assumptions.data()) != Z3_L_FALSE) {
            return c; // done
        } else {
            Z3_ast_vector core = Z3_solver_get_unsat_core(ctx, solver);
            unsigned m = Z3_ast_vector_size(ctx, core);
            std::vector<Z3_ast> block_vars(m);
            unsigned k = 0;
            // update soft-constraints and aux_vars
            for (unsigned i = 0; i < n; i++) {
                // check whether assumption[i] is in the core or not
                for (unsigned j = 0; j < m; j++) {
                    if (assumptions[i] == Z3_ast_vector_get(ctx, core, j)) {
                        // assumption[i] is in the unsat core... so soft_cnstrs[i] is in the unsat core
                        Z3_ast block_var = Z3_mk_fresh_const(ctx, nullptr, ty);
                        Z3_ast new_aux_var = Z3_mk_fresh_const(ctx, nullptr, ty);
                        soft[i] = mk_binary_or(ctx, soft[i], block_var);
                        aux_vars[i] = new_aux_var;
                        block_vars[k] = block_var;
                        ++k;
                        // Add new constraint containing the block variable.
                        // Note that we are using the new auxiliary variable to be able to use it as an assumption.
                        Z3_solver_assert(ctx, solver, mk_binary_or(ctx, soft[i], new_aux_var) );
                        break;
                    }
                }

            }
            if (k > 1) {
                std::vector<Z3_ast> aux_array_1(k + 1);
                std::vector<Z3_ast> aux_array_2(k + 1);
                Z3_ast * aux_1 = aux_array_1.data();
                Z3_ast * aux_2 = aux_array_2.data();
                std::copy_n(block_vars.data(), k, aux_array_1.data());
                unsigned i = 1;
                for (; k > 1; ++i) {
                    assert (aux_1 != aux_2);
                    k = mk_adder_pairs(ctx, i, k, aux_1, aux_2);
                    std::swap(aux_1, aux_2);
                }
                assert_le_one(ctx, solver, i, aux_1);
            }
        }
    }
    return 0;
}

#endif // MAXSAT_HPP
