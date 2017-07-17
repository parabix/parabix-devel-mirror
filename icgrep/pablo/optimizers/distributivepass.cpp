#include "distributivepass.h"

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>

#include <pablo/boolean.h>
#include <pablo/branch.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_count.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_string.h>
#include <pablo/pe_var.h>
#include <pablo/pe_zeroes.h>
#include <pablo/builder.hpp>

#include <pablo/optimizers/pablo_simplifier.hpp>

#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/function_output_iterator.hpp>
#include <boost/functional/hash.hpp>

#include <set>
#include <unordered_set>

#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif

#include <boost/graph/strong_components.hpp>
#include <llvm/Support/raw_ostream.h>
#include <pablo/printer_pablos.h>

// #define PRINT_DEBUG

using namespace boost;
using namespace boost::container;
using namespace llvm;

namespace pablo {

using TypeId = pablo::PabloAST::ClassTypeId;

enum class State {
    Dead
    , Live
    , Modified
};

using UsageTime = size_t;

using VertexData = std::tuple<pablo::PabloAST *, TypeId, State, UsageTime>;

using OperandIndex = unsigned;

using Graph = adjacency_list<vecS, vecS, bidirectionalS, VertexData, OperandIndex>;
using Vertex = Graph::vertex_descriptor;
using in_edge_iterator = graph_traits<Graph>::in_edge_iterator;
using out_edge_iterator = graph_traits<Graph>::out_edge_iterator;

using DistributionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Vertex>;
using DistributionVertex = DistributionGraph::vertex_descriptor;

using Sequence = std::vector<Vertex>;
using Biclique = std::pair<Sequence, Sequence>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<Sequence, Sequence, Sequence>;
using DistributionSets = std::vector<DistributionSet>;

using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

struct PassContainer {

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief run
     *
     * Based on the knowledge that:
     *
     *  (ASSOCIATIVITY)    A ∧ (B ∧ C) ⇔ (A ∧ B) ∧ C ⇔ A ∧ B ∧ C   and   A ∨ (B ∨ C) ⇔ (A ∨ B) ∨ C ⇔ A ∨ B ∨ C
     *
     *  (IDENTITY)         A ∨ 0 ⇔ A   and   A ∧ 1 = A
     *
     *  (ANNULMENT)        A ∧ 0 ⇔ 0   and   A ∨ 1 = 1
     *
     *  (IDEMPOTENT)       A ∨ (A ∨ B) ⇔ A ∨ B   and   A ∧ (A ∧ B) ⇔ A ∧ B
     *
     *  (ABSORBTION)       A ∨ (A ∧ B) ⇔ A ∧ (A ∨ B) ⇔ A
     *
     *  (COMPLEMENT)       A ∨ ¬A ⇔ 1   and   A ∧ ¬A = 0
     *
     *  (DISTRIBUTIVITY)   (A ∧ B) ∨ (A ∧ C) ⇔ A ∧ (B ∨ C)   and   (A ∨ B) ∧ (A ∨ C) ⇔ A ∨ (B ∧ C)
     *
     *  (DE MORGAN'S)      ¬(A ∧ B) ⇔ ¬A ∨ ¬B   and   ¬(A ∨ B) ⇔ ¬A ∧ ¬B
     *
     * Try to simplify the equations and eliminate some of the unnecessary statements
     ** ------------------------------------------------------------------------------------------------------------- */
    bool run(PabloKernel * const kernel) {
        readAST(kernel->getEntryBlock());
        if (LLVM_LIKELY(simplifyGraph())) {
            rewriteAST(kernel->getEntryBlock());
            return true;
        }
        return false;
    }

protected:

    #if defined(PRINT_DEBUG)
    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief printGraph
     ** ------------------------------------------------------------------------------------------------------------- */
    void printGraph(const std::string & name, llvm::raw_ostream & out, std::vector<Vertex> restricted = {}) {

        const auto n = num_vertices(G);
        std::vector<unsigned> c(n);
        const auto components = strong_components(G, make_iterator_property_map(c.begin(), get(vertex_index, G), c[0]));

        std::vector<bool> show(n, false);
        if (LLVM_LIKELY(restricted.empty() && n == components)) {
            for (const auto u : make_iterator_range(vertices(G))) {
                show[u] = isLive(u);
            }
        } else {
            std::queue<Vertex> Q;
            for (const auto m : restricted) {
                if (m < n && isLive(m)) {
                    show[m] = true;
                    assert (Q.empty());
                    Q.push(m);
                    for (;;) {
                        const auto u = Q.front();
                        Q.pop();
                        for (auto e : make_iterator_range(in_edges(u, G))) {
                            const auto v = source(e, G);
                            if (show[v] || !isLive(v)) {
                                continue;
                            }
                            show[v] = true;
                            Q.push(v);
                        }
                        if (Q.empty()) {
                            break;
                        }
                    }
                    for (auto e : make_iterator_range(out_edges(m, G))) {
                        const auto v = target(e, G);
                        show[v] = isLive(v) ? true : show[v];
                    }
                }
            }
        }

        out << "digraph " << name << " {\n";
        for (auto u : make_iterator_range(vertices(G))) {

            if (show[u]) {

                out << "v" << u << " [label=\"" << u << ": ";
                TypeId typeId;
                PabloAST * expr;
                State state;

                std::tie(expr, typeId, state, std::ignore) = G[u];

                bool space = true;

                switch (typeId) {
                    case TypeId::And:
                        out << "(∧)";
                        break;
                    case TypeId::Or:
                        out << "(∨)";
                        break;
                    case TypeId::Xor:
                        out << "(⊕)";
                        break;
                    case TypeId::Not:
                        out << "(¬)";
                        break;
                    case TypeId::Zeroes:
                        out << "(⊥)";
                        break;
                    case TypeId::Ones:
                        out << "(⊤)";
                    default:
                        space = false;
                        break;
                }
                if (expr) {
                    if (space) {
                        out << ' ';
                    }
                    expr->print(out);
                }

                const bool error = !hasValidOperandIndicies(u);

                out << "\"";
                if (state == State::Modified) {
                    out << " style=dashed";
                }
                if (error) {
                    out << " color=red";
                } else if (isRegenerable(typeId)) {                   
                    out << " color=blue";
                }
                out << "];\n";
            }
        }
        for (auto e : make_iterator_range(edges(G))) {
            const auto s = source(e, G);
            const auto t = target(e, G);
            if (show[s] && show[t]) {
                const auto cyclic = (c[s] == c[t]);
                const auto nonAssoc = !isAssociative(getType(t));
                out << "v" << s << " -> v" << t;
                if (cyclic || nonAssoc) {
                    out << " [";
                    if (nonAssoc) {
                        out << " label=\"" << G[e] << "\"";
                    }
                    if (cyclic) {
                        out << " color=red";
                    }
                    out << "]";
                }
                out << ";\n";
            }
        }

        out << "}\n\n";
        out.flush();
    }
    #endif

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief readAST
     ** ------------------------------------------------------------------------------------------------------------- */
    void readAST(PabloBlock * const block) {
        for (Statement * stmt : *block) {
            addStatement(stmt);
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                readAST(cast<Branch>(stmt)->getBody());
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief rewriteAST
     *
     * The goal here is to rewrite the AST with the minimal amount of perturbation to the sequence of instructions
     * themselves. The exception is that associative instructions are regenerated w.r.t. their sequence in the AST
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t rewriteAST(PabloBlock * entry, size_t count = 0) {

        for (Statement * stmt : *entry) {

            const Vertex u = findVertex(stmt);
            const auto typeId = getType(u);

            if (isDead(u) || isRegenerable(typeId)) {
                continue;
            }

            #ifdef PRINT_DEBUG
            errs() << u << ") ";
            PabloPrinter::print(stmt, errs());
            errs() << "\n";
            #endif

            assert (isLive(u));
            assert (stmt->getClassTypeId() == typeId);
            assert (hasValidOperandIndicies(u));
            assert (in_degree(u, G) == stmt->getNumOperands());

            in_edge_iterator ei_begin, ei_end;
            std::tie(ei_begin, ei_end) = in_edges(u, G);
            auto ei = ei_begin;

            // For each associative operand, find the vertex that describes the operand in G
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {

                // Does the vertex have a value and, if so, does value dominate this statement?
                // If not, we need to regenerate it.
                for (;;) {
                    if (ei == ei_end) {
                        ei = ei_begin;
                    }
                    if (G[*ei] == i) {
                        break;
                    }
                    ++ei;
                }

                entry->setInsertPoint(stmt->getPrevNode());

                stmt->setOperand(i, regenerateIfNecessary(stmt, entry, source(*ei, G), count));
            }

            if (LLVM_UNLIKELY(typeId == TypeId::Assign)) {
                setLastUsageTime(findVertex(stmt->getOperand(0)), ++count);
            }
            setLastUsageTime(u, ++count);
            setValue(u, stmt);

            if (isa<Branch>(stmt)) {
                count = rewriteAST(cast<Branch>(stmt)->getBody(), count);
            }
        }

        return count;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief regenerateIfNecessary
     *
     * Does vertex (u) have a value and, if so, does value dominate the statement? If not, regenerate it.
     ** ------------------------------------------------------------------------------------------------------------- */
    PabloAST * regenerateIfNecessary(Statement * const stmt, PabloBlock * const entry, const Vertex u, size_t & count) {

        assert (isLive(u));
        const TypeId typeId = getType(u);
        PabloAST * value = isModified(u) ? nullptr : getValue(u);
        if (LLVM_LIKELY(!dominates(value, stmt))) {

            const auto n = in_degree(u, G);            

            #ifdef PRINT_DEBUG
            errs() << "   making " << u << "  " << n << "  " << (int)typeId << "\n";
            #endif

            for (auto e : make_iterator_range(in_edges(u, G))) {
                const auto v = source(e, G);
                regenerateIfNecessary(stmt, entry, v, count);
                assert (getValue(v));
                assert (dominates(getValue(v), stmt));
            }

            if (LLVM_LIKELY(isAssociative(typeId))) {

                assert (n >= 2);

                Vertex input[n];
                unsigned i = 0;
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    input[i++] = source(e, G);
                }

                std::sort(input, input + n, [this](const Vertex u, const Vertex v) {
                    return getLastUsageTime(u) < getLastUsageTime(v);
                });

                PabloBuilder builder(entry);
                value = getValue(input[0]);
                for (unsigned i = 1; i < n; ++i) {
                    PabloAST * const op = getValue(input[i]);
                    switch (typeId) {
                        case TypeId::And:
                            value = builder.createAnd(value, op);
                            break;
                        case TypeId::Or:
                            value = builder.createOr(value, op);
                            break;
                        case TypeId::Xor:
                            value = builder.createXor(value, op);
                            break;
                        default:
                            llvm_unreachable("impossible!");
                    }
                }

            } else if (n == 1) {
                assert (typeId == TypeId::Not);
                value = entry->createNot(getValue(first_source(in_edges(u, G))));
            } else if (n > 1) {

                PabloAST * op[n] = { nullptr };
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    const auto i = G[e];
                    assert (i < n);
                    assert (op[i] == nullptr);
                    op[i] = getValue(source(e, G));
                    assert (op[i]);
                }

                switch (typeId) {
                    case TypeId::Advance:
                        value = entry->createAdvance(op[0], op[1]);
                        break;
                    case TypeId::ScanThru:
                        value = entry->createScanThru(op[0], op[1]);
                        break;
                    case TypeId::MatchStar:
                        value = entry->createMatchStar(op[0], op[1]);
                        break;

                    default:
                        llvm_unreachable("cannot regenerate this non-associative statement!");
                }

            } else {
                assert (isConstant(typeId));
                if (typeId == TypeId::Zeroes) {
                    value = entry->createZeroes();
                } else {
                    value = entry->createOnes();
                }
            }

            setValue(u, value);
            setUnmodified(u);
        }
        // negations inherit the last usage time from their operand
        if (LLVM_UNLIKELY(typeId == TypeId::Not)) {
            setLastUsageTime(u, getLastUsageTime(first_source(in_edges(u, G))));
        } else {
            setLastUsageTime(u, ++count);
        }
        return value;
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief simplifyGraph
     ** ------------------------------------------------------------------------------------------------------------- */
    bool simplifyGraph() {
        bool modified = false;
repeat: getReverseTopologicalOrdering();
        compactGraph();
        if (applyDistributivityLaw()) {
            modified = true;
            goto repeat;
        }
        transitiveReduction();
        factorizeGraph();
        return modified;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getReverseTopologicalOrdering
     ** ------------------------------------------------------------------------------------------------------------- */
    void getReverseTopologicalOrdering() {

        struct PrePassInserter {
            PrePassInserter & operator=(const Vertex u) {
                if (LLVM_LIKELY(self.isLive(u))) {
                    assert(self.hasValidOperandIndicies(u));
                    if (LLVM_LIKELY(isImmutable(self.getType(u)) || out_degree(u, self.G) != 0)) {
                        self.ordering.push_back(u);
                    } else {
                        self.removeVertex(u);
                    }
                }
                return *this;
            }

            PrePassInserter(PassContainer & pc) : self(pc) { }
            PrePassInserter & operator*() { return *this; }
            PrePassInserter & operator++() { return *this; }
            PrePassInserter & operator++(int) { return *this; }

        public:
            PassContainer & self;
        };


        ordering.clear();
        ordering.reserve(num_vertices(G));
        topological_sort(G, PrePassInserter(*this));
        assert (!ordering.empty());
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief compactGraph
     ** ------------------------------------------------------------------------------------------------------------- */
    void compactGraph() {

        auto IdentityHash = [this](const Vertex u) {
            using value_of = std::underlying_type<TypeId>::type;
            const auto n = in_degree(u, G);
            Vertex operands[n];
            unsigned i = 0;
            for (auto e : make_iterator_range(in_edges(u, G))) {
                operands[i++] = source(e, G);
            }
            std::sort(operands, operands + n);
            size_t h = 0;
            boost::hash_combine(h, static_cast<value_of>(getType(u)));
            for (unsigned j = 0; j < n; ++j) {
                boost::hash_combine(h, operands[j]);
            }
            return h;
        };

        auto IdentityComparator = [this](const Vertex u, const Vertex v) {
            const auto typeId = getType(u);
            if (LLVM_LIKELY(typeId == getType(v))) {
                const unsigned n = in_degree(u, G);
                if (LLVM_UNLIKELY(n == 0)) {
                    assert (isConstant(typeId) && in_degree(v, G) == 0);
                    return true;
                }
                if (in_degree(v, G) == n) {
                    Vertex adjA[n];
                    Vertex adjB[n];
                    auto ei = std::get<0>(in_edges(u, G));
                    auto ej = std::get<0>(in_edges(v, G));
                    // if this is an associative op, order doesn't matter
                    if (isAssociative(typeId)) {
                        for (unsigned i = 0; i < n; ++i, ++ei, ++ej) {
                            adjA[i] = source(*ei, G);
                            adjB[i] = source(*ej, G);
                        }
                        std::sort(adjA, adjA + n);
                        std::sort(adjB, adjB + n);
                    } else { // otherwise consider the order indicated by the edges
                        for (unsigned i = 0; i < n; ++i, ++ei, ++ej) {
                            adjA[G[*ei]] = source(*ei, G);
                            adjB[G[*ej]] = source(*ej, G);
                        }
                    }
                    for (unsigned i = 0; i < n; ++i) {
                        if (adjA[i] != adjB[i]) {
                            return false;
                        }
                    }
                    return true;
                }
            }
            return false;
        };

        using IdentitySet = std::unordered_set<Vertex, decltype(IdentityHash), decltype(IdentityComparator)>;

        IdentitySet V{0, IdentityHash, IdentityComparator};
        V.reserve(num_vertices(G));

        for (const auto u : boost::adaptors::reverse(ordering)) {

            assert (isLive(u));

            const auto typeId = getType(u);
            if (LLVM_UNLIKELY(isImmutable(typeId))) {
                continue;
            } else if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
                removeVertex(u);
                continue;
            }

            assert (hasValidOperandIndicies(u));

            if (LLVM_UNLIKELY(isConstant(typeId))) {
                if (processConstant(u, typeId)) {
                    continue;
                }
            } else if (isAssociative(typeId)) {
                if (processAssociative(u, typeId)) {
                    continue;
                }
            } else if (typeId == TypeId::Not) {
                if (processNegation(u, typeId)) {
                    continue;
                }
            }

            // check whether this vertex is a duplicate
            auto f = V.insert(u);
            if (LLVM_UNLIKELY(!f.second)) {
                remapVertex(u, *f.first);              
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief processAssociative
     ** ------------------------------------------------------------------------------------------------------------- */
    bool processAssociative(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (getType(u) == typeId);
        assert (isAssociative(typeId));

        if (LLVM_UNLIKELY(in_degree(u, G) == 0)) {
            #ifdef PRINT_DEBUG
            errs() << "nullary associative " << u << "\n";
            #endif
            setModified(u);
            setType(u, TypeId::Zeroes);
            return processConstant(u, TypeId::Zeroes);
        } else if (LLVM_UNLIKELY(in_degree(u, G) == 1)) {
            // An associative operation with only one element is always equivalent to the element
            const auto v = first_source(in_edges(u, G));
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                addEdge(v, target(e, G), G[e]);
            }
            #ifdef PRINT_DEBUG
            errs() << "unary associative " << v << " -> " << u << "\n";
            #endif
            removeVertex(u);
            return true;
        } else {
            // Take the transitive closure of these arcs to reveal the underlying equations
            Vertex removed[out_degree(u, G)];
            unsigned n = 0;
            for (auto ei : make_iterator_range(out_edges(u, G))) {
                const auto v = target(ei, G);
                if (typeId == getType(v)) {
                    assert(hasValidOperandIndicies(v));
                    for (auto ej : make_iterator_range(in_edges(u, G))) {
                        addEdge(source(ej, G), v, G[ei]);
                    }
                    setModified(v);
                    removed[n++] = v;
                    #ifdef PRINT_DEBUG
                    errs() << "transitive associative " << v << " -> " << u << "\n";
                    #endif
                }
            }
            for (unsigned i = 0; i < n; ++i) {
                const auto v = removed[i];
                assert (edge(u, v, G).second);
                remove_edge(u, v, G);
                assert(hasValidOperandIndicies(v));
            }
            if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
                removeVertex(u);
                return true;
            }
            if (LLVM_UNLIKELY(typeId == TypeId::Xor)) {
                // Canonicalize xor operations: (A ⊕ ¬B) = (A ⊕ B ⊕ 1)
                Vertex negation[in_degree(u, G)];
                unsigned count = 0;
                for (const auto e : make_iterator_range(in_edges(u, G))) {
                    const auto v = source(e, G);
                    if (LLVM_UNLIKELY(getType(v) == TypeId::Not)) {
                        negation[count++] = v;
                    }
                }
                if (LLVM_UNLIKELY(count != 0)) {
                    #ifdef PRINT_DEBUG
                    errs() << "xor canonicalization (a) " << u << "\n";
                    #endif
                    for (unsigned i = 0; i < count; ++i) {
                        const auto v = negation[i];
                        assert (edge(v, u, G).second);
                        remove_edge(v, u, G);
                        addEdge(first_source(in_edges(v, G)), u);
                    }
                    if ((count & 1) != 0) {
                        addEdge(makeVertex(TypeId::Ones), u);
                    }
                    setModified(u);
                    assert(hasValidOperandIndicies(u));
                }
            } else { // perform De Morgan's law expansion
                applyDeMorgans(u, typeId);
                return applyAbsorbtionComplementLaw(u, typeId);
            }
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief processNegation
     ** ------------------------------------------------------------------------------------------------------------- */
    bool processNegation(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert ("negation must have one input!" && in_degree(u, G) == 1);
        assert (getType(u) == typeId);
        assert (typeId == TypeId::Not);

        const auto v = first_source(in_edges(u, G));
        const auto negatedTypeId = getType(v);
        if (LLVM_UNLIKELY(negatedTypeId == TypeId::Not)) { // ¬¬A = A
            const auto w = first_source(in_edges(v, G));
            #ifdef PRINT_DEBUG
            errs() << "double negation " << u << " -> " << w << "\n";
            #endif
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                addEdge(w, v, G[e]);
                setModified(v);
            }
            removeVertex(u);
            return true;
        } else if (LLVM_UNLIKELY(negatedTypeId == TypeId::Xor)) {
            // Canonicalize xor operations: ¬(A ⊕ B) = (A ⊕ B ⊕ 1)
            #ifdef PRINT_DEBUG
            errs() << "xor canonicalization (n) " << u << "\n";
            #endif
            setModified(u);
            setType(u, TypeId::Xor);
            clear_in_edges(u, G);
            for (const auto e : make_iterator_range(in_edges(v, G))) {
                add_edge(source(e, G), u, 0, G);
            }
            addEdge(makeVertex(TypeId::Ones), u);
            assert(hasValidOperandIndicies(u));
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyDeMorgans
     ** ------------------------------------------------------------------------------------------------------------- */
    void applyDeMorgans(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (in_degree(u, G) > 0);
        assert (getType(u) == typeId);
        assert (isDistributive(typeId));

        Vertex A[in_degree(u, G)];
        unsigned n = 0;
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            if (LLVM_UNLIKELY(getType(v) == TypeId::Not)) {
                const auto w = first_source(in_edges(v, G));
                if (LLVM_UNLIKELY(getType(w) == oppositeTypeId(typeId))) {
                    A[n++] = v;
                }
            }
        }

        if (LLVM_UNLIKELY(n != 0)) {
            for (unsigned i = 0; i < n; ++i) {
                const auto v = A[i];
                #ifdef PRINT_DEBUG
                errs() << "de morgan's expansion " << v << " -> " << u << "\n";
                #endif
                assert (edge(v, u, G).second);
                assert (getType(v) == TypeId::Not);
                remove_edge(v, u, G);
                // NOTE: we cannot remove v even if this was its last edge since
                // it must be in our duplicate check map
                const auto w = first_source(in_edges(v, G));
                assert (getType(w) == oppositeTypeId(typeId));
                for (const auto e : make_iterator_range(in_edges(w, G))) {
                    const auto x = makeVertex(TypeId::Not);
                    add_edge(source(e, G), x, 0, G);
                    addEdge(x, u);
                }
            }
            setModified(u);
            assert(hasValidOperandIndicies(u));
        }

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAbsorbtionComplementLaw
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyAbsorbtionComplementLaw(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (in_degree(u, G) > 0);
        assert (getType(u) == typeId);
        assert (isDistributive(typeId));

        Vertex A[in_degree(u, G)];
        unsigned n = 0;
        for (const auto ei : make_iterator_range(in_edges(u, G))) {
            const auto v = source(ei, G);
            assert (isLive(v));
            const auto innerTypeId = getType(v);
            if (innerTypeId == TypeId::Not) {
                const auto w = first_source(in_edges(v, G));
                assert ("G is cyclic!" && (w != v));
                assert (isLive(w));
                for (const auto ej : make_iterator_range(in_edges(u, G))) {
                    if (LLVM_UNLIKELY(source(ej, G) == w)) {
                        const auto complementTypeId = (typeId == TypeId::And) ? TypeId::Zeroes : TypeId::Ones;
                        #ifdef PRINT_DEBUG
                        errs() << "complement (" << (int)complementTypeId << ") " << u << "\n";
                        #endif
                        setModified(u);
                        setType(u, complementTypeId);
                        clear_in_edges(u, G);
                        return processConstant(u, complementTypeId);
                    }
                }
            } else if (innerTypeId == oppositeTypeId(typeId) && LLVM_UNLIKELY(absorbs(u, v))) {
                A[n++] = v;
            }
        }

        if (n) {
            setModified(u);
            for (;;) {
                const auto v = A[--n];
                #ifdef PRINT_DEBUG
                errs() << "absorbing " << v  << "," << u << "\n";
                #endif
                assert (edge(v, u, G).second);
                remove_edge(v, u, G);
                if (LLVM_UNLIKELY(out_degree(v, G) == 0)) {
                    removeVertex(v);
                }
                if (LLVM_LIKELY(n == 0)) {
                    break;
                }
            }
        }

        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief absorbs
     ** ------------------------------------------------------------------------------------------------------------- */
    bool absorbs(const Vertex u, const Vertex v) {
        assert (getType(u) == oppositeTypeId(getType(v)));
        for (const auto ei : make_iterator_range(in_edges(u, G))) {
            for (const auto ej : make_iterator_range(in_edges(v, G))) {
                if (LLVM_UNLIKELY(source(ei, G) == source(ej, G))) {
                    return true;
                }
            }
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief processConstant
     ** ------------------------------------------------------------------------------------------------------------- */
    bool processConstant(const Vertex u, const TypeId typeId) {

        const auto l = out_degree(u, G);
        Vertex modification[l];
        unsigned n = 0;
        unsigned m = 0;

        assert (isConstant(typeId));
        assert (getType(u) == typeId);

        for (auto e : make_iterator_range(out_edges(u, G))) {
            const auto v = target(e, G);
            const auto targetTypeId = getType(v);
            if (LLVM_UNLIKELY(isDistributive(targetTypeId))) {
                // typeId           targetTypeId     Optimization
                // ------------     ------------     ------------
                // Zeroes           Or               identity
                // Zeroes           And              annulment (0)
                // Ones             Or               annulment (1)
                // Ones             And              identity
                if (isIdentityRelation(typeId, targetTypeId)) {
                    #ifdef PRINT_DEBUG
                    errs() << "identity (" << (int)(typeId) << ") " << v << " >> " << u << "\n";
                    #endif
                    modification[l - ++n] = v;
                } else { // annulment
                    #ifdef PRINT_DEBUG
                    errs() << "annulment (" << (int)(typeId) << ") " << u << " >> " << v << "\n";
                    #endif
                    setType(v, typeId);
                    modification[m++] = v;
                }
            } else if (LLVM_UNLIKELY(targetTypeId == TypeId::Not)) {
                const auto negatedTypeId = (typeId == TypeId::Zeroes) ? TypeId::Ones : TypeId::Zeroes;
                #ifdef PRINT_DEBUG
                errs() << "constant negation (" << (int)typeId << ") " << u << "\n";
                #endif
                setType(u, negatedTypeId);
                modification[m++] = v;
            } else if (LLVM_UNLIKELY(isStreamOperation(typeId))) {
                if (LLVM_LIKELY(typeId == TypeId::Zeroes)) {
                    #ifdef PRINT_DEBUG
                    errs() << "zero propagation (" << (int)(typeId) << ") " << u << " >> " << v << "\n";
                    #endif
                    setType(v, TypeId::Zeroes);
                    modification[m++] = v;
                } else { // if (typeId == TypeId::Ones) {
                    switch (targetTypeId) {
                        case TypeId::ScanThru:
                            if (LLVM_UNLIKELY(G[e] == 1)) {
                                setType(v, TypeId::Zeroes);
                                modification[m++] = v;
                            }
                            break;
                        case TypeId::MatchStar:
                            if (LLVM_UNLIKELY(G[e] == 0)) {
                                setType(v, TypeId::Ones);
                                modification[m++] = v;
                            }
                            break;
                        default: break;
                    }
                }
            }
        }

        if (LLVM_LIKELY(n == 0 && m == 0)) {
            return false;
        }

        // handle any identity graph modifications
        while (LLVM_UNLIKELY(n != 0)) {
            const auto v = modification[l - n--];
            setModified(v);
            remove_edge(u, v, G);
        }

        // ... then handle the rest
        while (LLVM_LIKELY(m != 0)) {
            const auto v = modification[--m];
            setModified(v);
            clear_in_edges(v, G);
            // We could recursively call "processConstant" but this could cause a stack overflow
            // in a pathological case. Instead rely on the fact v will be processed eventually by
            // the outer loop.
        }

        if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
            removeVertex(u);
            return true;
        }

        return false;
    }


    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyDistributivityLaw
     *
     * sources        (s)               inner       (∨)   (∨)
     *               /   \                            \   /
     * inner       (∨)   (∨)            x              (∧)
     *               \   /                              |
     * outer          (∧)               sources         |  (s)
     *                                                  | /
     *                                  y              (∨)
     *                                                  |
     *                                  outer          (∧)
     *
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyDistributivityLaw() {

        makeDistributionSubgraph();

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (LLVM_UNLIKELY(distributable.empty())) {
            return false;
        }

        bool modified = false;

        const auto lowerSet = obtainDistributableClauses(distributable);
        for (const auto & lower : lowerSet) {
            const auto & outer = std::get<0>(lower);
            const auto upperSet = obtainDistributableSources(std::get<1>(lower));
            for (const auto & upper : upperSet) {
                const auto & inner = std::get<0>(upper);
                const auto & sources = std::get<1>(upper);

                const auto outerTypeId = getType(Gd[outer[0]]);
                const auto innerTypeId = oppositeTypeId(outerTypeId);

                // Update G to match the desired change
                const auto x = makeVertex(outerTypeId);
                const auto y = makeVertex(innerTypeId);

                #ifdef PRINT_DEBUG
                errs() << "distributing {";
                print_dist(errs(), sources);
                if (innerTypeId == TypeId::And) {
                    errs() << "} ∧ {";
                } else {
                    errs() << "} ∨ {";
                }
                print_dist(errs(), inner);
                if (outerTypeId == TypeId::Or) {
                    errs() << "} ∨ {";
                } else {
                    errs() << "} ∧ {";
                }
                print_dist(errs(), outer);
                errs() << "} -> " << x << ',' << y << '\n';
                #endif

                for (const Vertex i : sources) {
                    const auto u = Gd[i];
                    for (const Vertex j : inner) {
                        const auto v = Gd[j];
                        assert (edge(u, v, G).second);
                        assert (getType(v) == innerTypeId);
                        remove_edge(u, v, G);
                    }
                    addEdge(u, y);
                }

                for (const auto i : inner) {
                    const auto u = Gd[i];
                    assert (getType(u) == innerTypeId);
                    for (const Vertex j : outer) {
                        const auto v = Gd[j];
                        assert (edge(u, v, G).second);
                        assert (getType(v) == outerTypeId);
                        remove_edge(u, v, G);
                    }
                    addEdge(u, x);
                }

                addEdge(x, y);

                for (const Vertex i : outer) {
                    const auto u = Gd[i];
                    assert (getType(u) == outerTypeId);
                    setModified(u);
                    addEdge(y, u);
                }

                modified = true;
            }
        }

        Gd.clear();
        Md.clear();

        return modified;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief makeDistributionSubgraph
     ** ------------------------------------------------------------------------------------------------------------- */
    void makeDistributionSubgraph() {
        // TODO: instead of creating a subgraph, mark the vertices in G as being part of the subgraph? The last usage
        // time could be a "round counter"
        distributable.clear();
        for (const auto u : boost::adaptors::reverse(ordering)) {
            if (LLVM_LIKELY(isLive(u) && out_degree(u, G) != 0)) {
                const auto outerTypeId = getType(u);
                if (isDistributive(outerTypeId)) {
                    const auto n = in_degree(u, G);
                    assert (n > 1);
                    const auto innerTypeId = oppositeTypeId(getType(u));
                    Vertex D[n];
                    unsigned count = 0;
                    for (const auto ei : make_iterator_range(in_edges(u, G))) {
                        const auto v = source(ei, G);
                        assert (isLive(v));
                        if (getType(v) == innerTypeId) {
                            bool safe = true;
                            for (const auto ej : make_iterator_range(out_edges(v, G))) {
                                const auto w = target(ej, G);
                                assert (isLive(w));
                                if (getType(w) != outerTypeId) {
                                    safe = false;
                                    break;
                                }
                            }
                            if (safe) {
                                D[count++] = v;
                            }
                        }
                    }
                    if (count > 1) {
                        const auto du = addDistributionVertex(u);
                        for (unsigned i = 0; i < count; ++i) {
                            const auto v = D[i];
                            assert (isLive(v));
                            if (getType(v) == innerTypeId) {
                                const auto dv = addDistributionVertex(v);
                                add_edge(dv, du, Gd);
                                for (const auto ej : make_iterator_range(in_edges(v, G))) {
                                    const auto w = source(ej, G);
                                    assert (isLive(w));
                                    add_edge(addDistributionVertex(w), dv, Gd);
                                }
                            }
                        }
                        distributable.push_back(du);
                    }
                }
            }
        }
        std::sort(distributable.begin(), distributable.end());
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief obtainDistributableClauses
     ** ------------------------------------------------------------------------------------------------------------- */
    BicliqueSet obtainDistributableClauses(const Sequence & S) {
        auto cliques = enumerateBicliques(S, Gd, 1, 2);
        // remove any cliques from our set that do not contain all of their users
        for (auto ci = cliques.begin(); ci != cliques.end(); ) {
            const auto & A = std::get<0>(*ci);
            auto & B = std::get<1>(*ci);
            for (auto bi = B.begin(); bi != B.end(); ) {
                if (out_degree(Gd[*bi], G) != A.size()) {
                    bi = B.erase(bi);
                } else {
                    ++bi;
                }
            }
            if (B.size() < 2) {
                ci = cliques.erase(ci);
            } else {
                ++ci;
            }
        }
        return makeIndependent(std::move(cliques), 0);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief obtainDistributableSources
     ** ------------------------------------------------------------------------------------------------------------- */
    BicliqueSet obtainDistributableSources(const Sequence & S) {
        return makeIndependent(enumerateBicliques(S, Gd, 2, 1), 0);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief transitiveReduction
     ** ------------------------------------------------------------------------------------------------------------- */
    void transitiveReduction() {
        flat_set<Vertex> T;
        for (const auto u : ordering) {
            if (isLive(u)) {
                const auto typeId = getType(u);
                if (isAssociative(typeId)) {
                    assert (in_degree(u, G) != 0);
                    for (auto ei : make_iterator_range(in_edges(u, G))) {
                        const auto v = source(ei, G);
                        assert (isLive(v));
                        if (getType(v) == typeId) {
                            for (auto ej : make_iterator_range(in_edges(v, G))) {
                                const auto w = source(ej, G);
                                assert (isLive(w));
                                T.insert(w);
                            }
                        }
                    }
                    #ifndef NDEBUG
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        assert (T.count(source(e, G)) == 0);
                    }
                    #endif
                    for (const auto w : T) {
                        remove_edge(w, u, G);
                    }
                    T.clear();
                    assert (in_degree(u, G) > 1);
                }
            }
        }
    }

    void print_dist(raw_ostream & out, const Sequence & S) {
        if (S.empty()) {
            return;
        }
        out << Gd[S[0]];
        for (unsigned i = 1; i < S.size(); ++i) {
            out << ',' << Gd[S[i]];
        }
    }

    void print(raw_ostream & out, const Sequence & S) {
        if (S.empty()) {
            return;
        }
        out << S[0];
        for (unsigned i = 1; i < S.size(); ++i) {
            out << ',' << S[i];
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief factorizeGraph
     *
     * Factorize the associative operations in the final graph
     ** ------------------------------------------------------------------------------------------------------------- */
    void factorizeGraph() {

        Sequence groups[3];
        for (const auto u : make_iterator_range(vertices(G))) {
            if (isLive(u)) {
                switch (getType(u)) {
                    case TypeId::And:
                        groups[0].push_back(u); break;
                    case TypeId::Or:
                        groups[1].push_back(u); break;
                    case TypeId::Xor:
                        groups[2].push_back(u); break;
                    default: break;
                }
            }
        }

        const TypeId op[3] = { TypeId::And, TypeId::Or, TypeId::Xor };

        for (unsigned i = 0; i < 3; ++i) {

            // Although we risk losing better combinations by greedily taking the larger factorings,
            // choosing only those of minSize or greater first can significantly reduce the running
            // time of this optimization.

            for (unsigned minSize = 64; minSize >= 2; minSize /= 2)  {

                for (;;) {

                    #ifdef PRINT_DEBUG
                    errs() << "--------------------------------------------\n";
                    #endif

                    auto factors = makeIndependent(enumerateBicliques(groups[i], G, 2, minSize), 1);
                    if (factors.empty()) {
                        break;
                    }

                    for (auto & factor : factors) {
                        const auto & sources = std::get<1>(factor);
                        assert (sources.size() > 1);
                        auto & targets = std::get<0>(factor);
                        assert (targets.size() > 1);

                        #ifdef PRINT_DEBUG
                        errs() << "factoring {";
                        print(errs(), sources);
                        switch (op[i]) {
                            case TypeId::And: errs() << "} ∧ {"; break;
                            case TypeId::Or: errs() << "} ∨ {"; break;
                            case TypeId::Xor: errs() << "} ⊕ {"; break;
                            default: llvm_unreachable("impossible");
                        }
                        print(errs(), targets);
                        errs() << "} -> ";
                        #endif

                        // Check whether one of the targets is the factorization
                        Vertex x = 0;
                        bool create = true;
                        for (auto j = targets.begin(); j != targets.end(); ) {
                            assert (hasValidOperandIndicies(*j));
                            if (in_degree(*j, G) == sources.size()) {
                                if (LLVM_LIKELY(create)) {
                                    x = *j;
                                    create = false;
                                } else {
                                    for (auto e : make_iterator_range(out_edges(*j, G))) {
                                        addEdge(x, target(e, G), G[e]);
                                    }
                                    removeVertex(*j);
                                }
                                j = targets.erase(j);
                            } else {
                                ++j;
                            }
                        }
                        if (create) {
                            x = makeVertex(op[i]);
                            groups[i].push_back(x);
                            for (auto u : sources) {
                                add_edge(u, x, 0, G);
                            }
                            assert (hasValidOperandIndicies(x));
                        }
                        #ifdef PRINT_DEBUG
                        errs() << x << '\n';
                        #endif

                        // Remove the biclique between the source and target vertices
                        for (auto u : sources) {
                            for (auto v : targets) {
                                assert (getType(v) == op[i]);
                                assert (edge(u, v, G).second);
                                boost::remove_edge(u, v, G);
                            }
                        }

                        // ... and replace it with the factorization
                        for (auto v : targets) {
                            assert (getType(v) == op[i]);
                            addEdge(x, v);
                            setModified(v);
                            assert(hasValidOperandIndicies(v));
                        }
                    }
                }
            }
        }
    }

private:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief enumerateBicliques
     *
     * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
     * bicliques" by Alexe et. al. (2003). This implementation considers all verticies in set S to be in
     * bipartition A (0) and their *INCOMING* adjacencies to be in B (1).
     ** ------------------------------------------------------------------------------------------------------------- */
    template <typename Graph>
    BicliqueSet enumerateBicliques(const Sequence & S, const Graph & G, const unsigned minimumSizeA = 1, const unsigned minimumSizeB = 1) {
        using IntersectionSets = std::set<Sequence>;

        assert (std::is_sorted(S.begin(), S.end()));

        BicliqueSet cliques(0);

        if (LLVM_LIKELY(S.size() >= minimumSizeA)) {

            IntersectionSets B1;
            for (auto u : S) {
                const auto n = in_degree(u, G);
                if (n >= minimumSizeB) {
                    Sequence B;
                    B.reserve(n);
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        const auto v = source(e, G);
                        if (out_degree(v, G) >= minimumSizeA) {
                            B.push_back(v);
                        }
                    }
                    if (B.size() >= minimumSizeB) {
                        std::sort(B.begin(), B.end());
                        assert(std::unique(B.begin(), B.end()) == B.end());
                        B1.insert(std::move(B));
                    }
                }
            }

            IntersectionSets B(B1);

            IntersectionSets Bi;

            Sequence T;
            for (auto i = B1.begin(), end = B1.end(); i != end; ++i) {
                assert (std::is_sorted(i->begin(), i->end()));
                for (auto j = i; ++j != end; ) {
                    assert (std::is_sorted(j->begin(), j->end()));
                    std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(T));
                    if ((T.size() >= minimumSizeB) && (B.count(T) == 0)) {
                        Bi.insert(T);
                    }
                    T.clear();
                }
            }

            IntersectionSets Bj;
            for (;;) {
                if (Bi.empty()) {
                    break;
                }
                B.insert(Bi.begin(), Bi.end());
                for (auto i = B1.begin(); i != B1.end(); ++i) {
                    assert (std::is_sorted(i->begin(), i->end()));
                    for (auto j = Bi.begin(), end = Bi.end(); j != end; ++j) {
                        assert (std::is_sorted(j->begin(), j->end()));
                        std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(T));
                        if ((T.size() >= minimumSizeB) && (B.count(T) == 0)) {
                            Bj.insert(T);
                        }
                        T.clear();
                    }
                }
                Bi.swap(Bj);
                Bj.clear();
            }

            cliques.reserve(B.size());

            Sequence Aj;
            for (auto && Bi : B) {
                Sequence Ai(S);
                assert (Bi.size() >= minimumSizeB);
                bool largeEnough = true;
                for (const Vertex u : Bi) {
                    assert (std::is_sorted(Ai.begin(), Ai.end()));
                    assert (Ai.size() >= minimumSizeA);
                    T.clear();
                    const auto m = out_degree(u, G);
                    assert (m >= minimumSizeA);
                    T.reserve(m);
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        T.push_back(target(e, G));
                    }
                    std::sort(T.begin(), T.end());
                    assert(std::unique(T.begin(), T.end()) == T.end());
                    Aj.clear();
                    Aj.reserve(std::min(Ai.size(), T.size()));
                    std::set_intersection(Ai.begin(), Ai.end(), T.begin(), T.end(), std::back_inserter(Aj));
                    if (Aj.size() < minimumSizeA) {
                        largeEnough = false;
                        break;
                    }
                    Ai.swap(Aj);
                }
                if (largeEnough) {
                    cliques.emplace_back(std::move(Ai), std::move(Bi));
                }
            }

        }

        return cliques;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief makeIndependent
     ** ------------------------------------------------------------------------------------------------------------- */
    BicliqueSet && makeIndependent(BicliqueSet && S, const unsigned independentSide) {

        const auto l = S.size();
        IndependentSetGraph I(l);

        assert (independentSide < 2);

        // Initialize our weights
        for (unsigned i = 0; i != l; ++i) {
            I[i] = std::get<0>(S[i]).size() * std::get<1>(S[i]).size();
        }

        // Determine our constraints
        for (unsigned i = 0; i != l; ++i) {
            const auto & Si = (independentSide == 0) ? std::get<0>(S[i]) : std::get<1>(S[i]);
            for (unsigned j = i + 1; j != l; ++j) {
                const auto & Sj = (independentSide == 0) ? std::get<0>(S[j]) : std::get<1>(S[j]);
                if (intersects(Si, Sj)) {
                    boost::add_edge(i, j, I);
                }
            }
        }

        // Use the greedy algorithm to choose our independent set
        Sequence selected;
        for (;;) {
            unsigned w = 0;
            Vertex u = 0;
            for (unsigned i = 0; i != l; ++i) {
                if (I[i] > w) {
                    w = I[i];
                    u = i;
                }
            }
            if (LLVM_UNLIKELY(w == 0)) break;
            selected.push_back(u);
            I[u] = 0;
            for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
                I[v] = 0;
            }
        }

        // Sort the selected list and then remove the unselected cliques
        std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
        auto end = S.end();
        for (const unsigned offset : selected) {
            end = S.erase(S.begin() + offset + 1, end) - 1;
        }
        S.erase(S.begin(), end);

        return std::move(S);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief makeVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex makeVertex(const TypeId typeId, PabloAST * const expr = nullptr) {
        return add_vertex(std::make_tuple(expr, typeId, expr ? State::Live : State::Modified, 0), G);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addExpression
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex addExpression(PabloAST * const expr) {
        assert (expr);
        const auto f = M.find(expr);
        if (LLVM_LIKELY(f != M.end())) {
            return f->second;
        }
        assert (isConstant(expr->getClassTypeId()) || isLiteral(expr->getClassTypeId()));
        const auto u = makeVertex(expr->getClassTypeId(), expr);
        M.emplace(expr, u);
        return u;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addStatement
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex addStatement(Statement * const stmt) {
        assert (stmt);
        assert (M.count(stmt) == 0);
        const auto typeId = stmt->getClassTypeId();
        if (LLVM_UNLIKELY(typeId == TypeId::Sel)) {
            const auto c = addExpression(cast<Sel>(stmt)->getCondition());
            const auto t = addExpression(cast<Sel>(stmt)->getTrueExpr());
            const auto f = addExpression(cast<Sel>(stmt)->getFalseExpr());
            const auto u = makeVertex(TypeId::And);
            add_edge(c, u, 0, G);
            add_edge(t, u, 1, G);
            const auto n = makeVertex(TypeId::Not);
            add_edge(c, n, 0, G);
            const auto v = makeVertex(TypeId::And);
            add_edge(n, v, 0, G);
            add_edge(f, v, 1, G);
            const auto w = makeVertex(TypeId::Or);
            add_edge(u, w, 0, G);
            add_edge(v, w, 1, G);
            M.emplace(stmt, w);
            return w;
        } else {
            const auto u = makeVertex(typeId, stmt);
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<String>(op))) {
                    continue;
                }
                add_edge(addExpression(op), u, i, G);
            }
            assert(hasValidOperandIndicies(u));
            M.emplace(stmt, u);
            return u;
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addEdge
     ** ------------------------------------------------------------------------------------------------------------- */
    bool addEdge(const Vertex u, const Vertex v, const OperandIndex index = 0) {
        const auto typeId = getType(v);
        if (isAssociative(typeId)) {
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                if (LLVM_UNLIKELY(target(e, G) == v)) {
                    if (LLVM_LIKELY(isDistributive(typeId))) {
                        G[e] = std::max(G[e], index);
                    } else {
                        remove_edge(e, G);
                    }
                    return false;
                }
            }
        }
        boost::add_edge(u, v, index, G);
        return true;
    }

    #if !defined(NDEBUG) || defined(PRINT_DEBUG)
    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief hasValidOperandIndicies
     ** ------------------------------------------------------------------------------------------------------------- */
    bool hasValidOperandIndicies(const Vertex u, const bool report = true) {
        if (isLive(u)) {
            const auto n = in_degree(u, G);
            const auto typeId = getType(u);
            if (LLVM_UNLIKELY(n == 0)) {
                if (LLVM_LIKELY(isAssociative(typeId) || isNullary(typeId))) {
                    return true;
                }
                if (report) {
                    errs() << u << " cannot be nullary " << (int)typeId << "\n";
                }
                return false;
            } else if (isAssociative(typeId)) {
                Vertex V[n];
                unsigned i = 0;
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    V[i++] = source(e, G);
                }
                std::sort(V, V + n);
                for (unsigned i = 1; i != n; ++i) {
                    if (LLVM_UNLIKELY(V[i - 1] == V[i])) {
                        if (report) {
                            errs() << u << " has duplicate operands " << V[i] << "\n";
                        }
                        return false;
                    }
                }
            } else if (requiredOperands(typeId) == n) {
                bool used[n] = { false };
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    const auto i = G[e];
                    if (LLVM_UNLIKELY(i >= n)) {
                        if (report) {
                            errs() << u << " has operand index " << i << " exceeds in degree " << n << "\n";
                        }
                        return false;
                    } else if (LLVM_UNLIKELY(used[i])) {
                        if (report) {
                            errs() << u << " has duplicate operand indicies " << i << "\n";
                        }
                        return false;
                    }
                    used[i] = true;
                }
            } else {
                if (report) {
                    errs() << u << " has " << n << " operands but requires " << requiredOperands(typeId) << "\n";
                }
                return false;
            }
        }
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            if (!isLive(v)) {
                if (report) {
                    errs() << u << " has dead operand " << v << "\n";
                }
                return false;
            }
        }
        return true;
    }

    static unsigned requiredOperands(const TypeId typeId) {
        switch (typeId) {
            case TypeId::Not:
            case TypeId::InFile:
            case TypeId::AtEOF:
            case TypeId::Count:
            case TypeId::If:
            case TypeId::While:
                return 1;
            case TypeId::Sel:
                llvm_unreachable("impossible");
            default:
                return 2;
        }
    }
    #endif

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief removeVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    void removeVertex(const Vertex u) {
        #ifdef PRINT_DEBUG
        errs() << "removing " << u << "\n";
        #endif
        assert (isLive(u));
        setDead(u);
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            if (LLVM_UNLIKELY(out_degree(v, G) == 1)) {
                removeVertex(v);
            }
        }
        clear_vertex(u, G);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief remapVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    void remapVertex(const Vertex u, const Vertex v) {
        #ifdef PRINT_DEBUG
        errs() << "remapping " << u << " -> " << v << "\n";
        #endif
        assert (u != v);
        assert (isLive(v));
        assert (isRegenerable(getType(u)) || getValue(u));
        if (PabloAST * expr = getValue(u)) {
            auto f = M.find(expr);
            assert (f->second == u);
            f->second = v;
        }
        for (auto e : make_iterator_range(out_edges(u, G))) {
            addEdge(v, target(e, G), G[e]);
        }
        removeVertex(u);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief findVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex findVertex(const PabloAST * const expr) const {
        assert (expr);
        const auto f = M.find(expr);
        assert (f != M.end());
        return f->second;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addDistributionVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    DistributionVertex addDistributionVertex(const Vertex u) {
        assert (isLive(u));
        const auto f = Md.find(u);
        if (f == Md.end()) {
            #ifndef NDEBUG
            for (auto v : make_iterator_range(vertices(Gd))) {
                assert ("duplicate vertex found that was not in Md!" && Gd[v] != u);
            }
            #endif
            const auto du = add_vertex(u, Gd);
            Md.emplace(u, du);
            return du;
        }
        return f->second;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief intersects
     ** ------------------------------------------------------------------------------------------------------------- */
    template <class Type>
    inline bool intersects(Type & A, Type & B) {
        auto first1 = A.begin(), last1 = A.end();
        assert (std::is_sorted(first1, last1));
        auto first2 = B.begin(), last2 = B.end();
        assert (std::is_sorted(first2, last2));
        while (first1 != last1 && first2 != last2) {
            if (*first1 < *first2) {
                ++first1;
            } else if (*first2 < *first1) {
                ++first2;
            } else {
                return true;
            }
        }
        return false;
    }

    TypeId getType(const Vertex u) const {
        assert (u < num_vertices(G));
        return std::get<1>(G[u]);
    }

    void setType(const Vertex u, const TypeId typeId) {
        assert (u < num_vertices(G));
        std::get<1>(G[u]) = typeId;
    }

    PabloAST * getValue(const Vertex u) const {
        assert (u < num_vertices(G));
        return std::get<0>(G[u]);
    }

    void setValue(const Vertex u, PabloAST * const value) {
        assert (u < num_vertices(G));
        std::get<0>(G[u]) = value;
    }

    bool isLive(const Vertex u) const {
        return getState(u) != State::Dead;
    }

    bool isDead(const Vertex u) const {
        return getState(u) == State::Dead;
    }

    void setDead(const Vertex u) {
        setState(u, State::Dead);
    }

    void setUnmodified(const Vertex u) {
        setState(u, State::Live);
    }

    bool isModified(const Vertex u) const {
        return getState(u) == State::Modified;
    }

    void setModified(const Vertex u) {
        assert(!isImmutable(getType(u)));
        setState(u, State::Modified);
    }

    State getState(const Vertex u) const {
        assert (u < num_vertices(G));
        return std::get<2>(G[u]);
    }

    void setState(const Vertex u, const State value) {
        assert (u < num_vertices(G));
        std::get<2>(G[u]) = value;
    }

    UsageTime getLastUsageTime(const Vertex u) const {
        assert (u < num_vertices(G));
        return std::get<3>(G[u]);
    }

    void setLastUsageTime(const Vertex u, const UsageTime time) {
        assert (u < num_vertices(G));
        std::get<3>(G[u]) = time;
    }

    static bool isIdentityRelation(const TypeId a, const TypeId b) {
        assert (isConstant(a) && isDistributive(b));
        return !((a == TypeId::Zeroes) ^ (b == TypeId::Or));
    }

    static bool isAssociative(const TypeId typeId) {
        return (isDistributive(typeId) || typeId == TypeId::Xor);
    }

    static bool isRegenerable(const TypeId typeId) {
        return (isConstant(typeId) || isAssociative(typeId) || typeId == TypeId::Not);
    }

    static bool isAssociative(const PabloAST * const expr) {
        return isAssociative(expr->getClassTypeId());
    }

    static bool isConstant(const TypeId typeId) {
        return typeId == TypeId::Zeroes || typeId == TypeId::Ones;
    }

    static bool isLiteral(const TypeId typeId) {
        return typeId == TypeId::Integer || typeId == TypeId::Var;
    }

    static bool isDistributive(const TypeId typeId) {
        return (typeId == TypeId::And || typeId == TypeId::Or);
    }

    static bool isImmutable(const TypeId typeId) {
        switch (typeId) {
            case TypeId::Extract: case TypeId::Assign: case TypeId::If: case TypeId::While:
                return true;
            default:
                return isLiteral(typeId);
        }
    }

    static bool isNullary(const TypeId typeId) {
        return isConstant(typeId) || isLiteral(typeId);
    }

    static bool isStreamOperation(const TypeId typeId) {
        switch (typeId) {
            case TypeId::Advance:
            case TypeId::ScanThru:
            case TypeId::AdvanceThenScanThru:
//            case TypeId::ScanTo:
//            case TypeId::AdvanceThenScanTo:
            case TypeId::Lookahead:
            case TypeId::MatchStar:
            case TypeId::InFile:
            case TypeId::AtEOF:
                return true;
            default:
                return false;
        }
    }

    static TypeId oppositeTypeId(const TypeId typeId) {
        assert (isDistributive(typeId));
        return (typeId == TypeId::And) ? TypeId::Or : TypeId::And;
    }

    Vertex first_source(const std::pair<in_edge_iterator, in_edge_iterator> & e) const {
        return source(*std::get<0>(e), G);
    }

    bool inCurrentScope(const PabloAST * const expr, const PabloBlock * const scope) {
        return isa<Statement>(expr) && cast<Statement>(expr)->getParent() == scope;
    }

private:

    Graph G;
    flat_map<const pablo::PabloAST *, Vertex> M;

    DistributionGraph Gd;
    flat_map<Vertex, DistributionVertex> Md;

    Sequence ordering;
    Sequence distributable;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool DistributivePass::optimize(PabloKernel * const kernel) {

    PassContainer C;
    C.run(kernel);
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-distributive-pass");
    #endif
    Simplifier::optimize(kernel);
    return true;
}

}
