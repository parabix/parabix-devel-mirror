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

#include <pablo/optimizers/pablo_simplifier.hpp>

#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/function_output_iterator.hpp>
#include <boost/functional/hash.hpp>

#include <set>
#include <random>

#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif

#include <boost/graph/strong_components.hpp>
#include <llvm/Support/raw_ostream.h>
#include <pablo/printer_pablos.h>
#include <llvm/Support/CommandLine.h>

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
     * Try to simplify the equations and eliminate some of the unnecessary statements
     ** ------------------------------------------------------------------------------------------------------------- */
    bool run(PabloKernel * const kernel) {
        readAST(kernel->getEntryBlock());
        if (simplifyGraph()) {
            rewriteAST(kernel->getEntryBlock());
            return true;
        }
        return false;
    }

    PassContainer()
    : R(std::random_device()()) {

    }

protected:

    #if !defined(NDEBUG) || defined(PRINT_DEBUG)
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
                if (m < n) {
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

                switch (typeId) {
                    case TypeId::And:
                        out << "(∧) ";
                        break;
                    case TypeId::Or:
                        out << "(∨) ";
                        break;
                    case TypeId::Xor:
                        out << "(⊕) ";
                        break;
                    case TypeId::Not:
                        out << "(¬) ";
                        break;
                    case TypeId::Zeroes:
                        out << "(0) ";
                        break;
                    case TypeId::Ones:
                        out << "(1) ";
                    default:
                        break;
                }

                if (expr) {
                    PabloPrinter::print(expr, out);
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

            if (isRegenerable(typeId)) {
                continue;
            }

            #ifdef PRINT_DEBUG
            errs() << u << ") ";
            PabloPrinter::print(stmt, errs());
            errs() << "\n";
            #endif

            #ifndef NDEBUG
            if (LLVM_UNLIKELY(in_degree(u, G) != stmt->getNumOperands())) {
                printGraph("E", errs());
                errs() << "in degree (" << in_degree(u, G) << ") of " << u << " does not match number of operands (" << stmt->getNumOperands() << ")\n";
            }
            #endif

            assert (stmt->getClassTypeId() == typeId);
            assert (in_degree(u, G) == stmt->getNumOperands());

            in_edge_iterator ei_begin, ei_end;
            std::tie(ei_begin, ei_end) = in_edges(u, G);
            auto ei = ei_begin;

            // For each associative operand, find the vertex that describes the operand in G
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {

                // Does the vertex have a value and, if so, does value dominate this statement?
                // If not, we need to regenerate it.
                for (bool first_cycle = true;;) {
                    if (ei == ei_end) {
                        assert (first_cycle);
                        ei = ei_begin;
                        first_cycle = false;
                    }
                    if (G[*ei] == i) {
                        break;
                    }
                    ++ei;
                }

                entry->setInsertPoint(stmt->getPrevNode());

                PabloAST * const replacement = regenerateIfNecessary(stmt, entry, source(*ei, G), count);
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(replacement == op)) {
                    continue;
                }

                #ifdef PRINT_DEBUG
                errs() << " " << source(*ei, G) << ") replacing ";
                op->print(errs());
                errs() << " with ";
                replacement->print(errs());
                errs() << "\n";
                #endif

                stmt->setOperand(i, replacement);
            }

            if (LLVM_UNLIKELY(typeId == TypeId::Assign)) {
                setLastUsageTime(findVertex(stmt->getOperand(0)), count);
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
        PabloAST * value = isModified(u) ? nullptr : getValue(u);
        if (LLVM_LIKELY(!dominates(value, stmt))) {

            const auto n = in_degree(u, G);
            const TypeId typeId = getType(u);

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
                value = getValue(input[0]);
                for (unsigned i = 1; i < n; ++i) {
                    PabloAST * const op = getValue(input[i]);
                    switch (typeId) {
                        case TypeId::And:
                            value = entry->createAnd(value, op);
                            break;
                        case TypeId::Or:
                            value = entry->createOr(value, op);
                            break;
                        case TypeId::Xor:
                            value = entry->createXor(value, op);
                            break;
                        default:
                            llvm_unreachable("impossible!");
                    }
                }

            } else if (n == 1) {
                assert (typeId == TypeId::Not);
                value = getValue(first_source(in_edges(u, G)));
                value = entry->createNot(value);

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
                        llvm_unreachable("cannot regenerate this non-associtive statement!");
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
        setLastUsageTime(u, ++count);
        return value;
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief simplifyGraph
     ** ------------------------------------------------------------------------------------------------------------- */
    bool simplifyGraph() {

        bool modified = false;

        #ifdef PRINT_DEBUG
        errs() << "********************************************\n";
        #endif

restart:

        #ifdef PRINT_DEBUG
        errs() << "============================================ (1)\n";
        #endif

        getReverseTopologicalOrdering();

        #ifdef PRINT_DEBUG
        errs() << "============================================ (2)\n";
        #endif

        if (applyAnnulmentAssociativeIdentityIdempotentLaws()) {
            modified = true;
            goto restart;
        }

        #ifdef PRINT_DEBUG
        errs() << "============================================ (3)\n";
        #endif


        if (applyAbsorbtionComplementLaws()) {
            modified = true;
            goto restart;
        }

        #ifdef PRINT_DEBUG
        errs() << "============================================ (4)\n";
        #endif

        if (applyDistributivityLaw()) {
            modified = true;
            goto restart;
        }

        if (modified) {

            #ifdef PRINT_DEBUG
            errs() << "============================================ (5)\n";
            #endif

            transitiveReduction();

            #ifdef PRINT_DEBUG
            errs() << "============================================ (6)\n";
            #endif

            factorizeGraph();

            #ifdef PRINT_DEBUG
            // printGraph("G", errs());
            #endif            
        }

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
     * @brief applyAnnulmentAssociativeIdentityIdempotentLaws
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyAnnulmentAssociativeIdentityIdempotentLaws() {

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

        bool modified = false;

        for (const auto u : boost::adaptors::reverse(ordering)) {
            assert (isLive(u));
            assert(hasValidOperandIndicies(u));

            auto typeId = getType(u);

            if (LLVM_UNLIKELY(isImmutable(typeId))) {
                continue;
            } else if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
                removeVertex(u);
                continue;
            }

            if (LLVM_UNLIKELY(isConstant(typeId))) { identity_or_annulment_check:

                assert (typeId == getType(u));

                const auto n = out_degree(u, G);
                Vertex T[n];
                unsigned count = 0;

                for (auto e : make_iterator_range(out_edges(u, G))) {
                    const auto v = target(e, G);
                    if (LLVM_UNLIKELY(isDistributive(getType(v)))) {
                        assert (count < n);
                        assert (u != v);
                        T[count++] = v;
                    }
                }

                while (LLVM_UNLIKELY(count-- != 0)) {

                    // typeId           targetTypeId     Optimization
                    // ------------     ------------     ------------
                    // Zeroes           Or               identity
                    // Zeroes           And              annulment (0)
                    // Ones             Or               annulment (1)
                    // Ones             And              identity

                    assert (count < n);
                    const auto v = T[count];
                    setModified(v);
                    modified = true;
                    if (isIdentityRelation(typeId, getType(v))) {
                        #ifdef PRINT_DEBUG
                        errs() << "identity " << v << " >> " << u << "\n";
                        #endif
                        assert (edge(u, v, G).second);
                        remove_edge(u, v, G);
                    } else { // annulment
                        #ifdef PRINT_DEBUG
                        errs() << "annulment (" << (int)(typeId) << ") " << u << " >> " << v << "\n";
                        #endif
                        setType(v, typeId);
                        clear_in_edges(v, G);
                    }
                }

            } else if (isAssociative(typeId)) {
                if (LLVM_UNLIKELY(in_degree(u, G) == 0)) {
                    #ifdef PRINT_DEBUG
                    errs() << "nullary associative " << u << "\n";
                    #endif
                    setModified(u);
                    typeId = TypeId::Zeroes;
                    setType(u, TypeId::Zeroes);
                    modified = true;
                    goto identity_or_annulment_check;
                } else if (LLVM_UNLIKELY(in_degree(u, G) == 1)) {
                    // An associative operation with only one element is always equivalent to the element
                    const auto v = first_source(in_edges(u, G));
                    for (const auto e : make_iterator_range(out_edges(u, G))) {
                        addEdge(v, target(e, G), G[e]);
                    }                   
                    #ifdef PRINT_DEBUG
                    errs() << "unary associative " << v << " >> " << u << "\n";
                    #endif
                    removeVertex(u);
                    continue;
                } else {
                    // Take the transitive closure of these arcs, we may reveal the underlying equation
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
                            errs() << "transitive associative " << v << " >> " << u << "\n";
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
                        continue;
                    }                    
                    if (LLVM_UNLIKELY(typeId == TypeId::Xor)) {
                        // Canonicalize xor operations: (A ⊕ ¬B) = (A ⊕ B ⊕ 1)
                        Vertex negation[in_degree(u, G)];
                        unsigned count = 0;
                        for (const auto e : make_iterator_range(in_edges(u, G))) {
                            const auto v = source(e, G);
                            const auto typeId = getType(v);
                            if (LLVM_UNLIKELY(typeId == TypeId::Not)) {
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
                            modified = true;
                        }
                    }
                }
            } else if (typeId == TypeId::Not) {
                assert (in_degree(u, G) == 1);
                const auto v = first_source(in_edges(u, G));
                const auto negatedTypeId = getType(v);
                if (LLVM_UNLIKELY(negatedTypeId == TypeId::Not)) {
                    // handle double negation
                    assert (in_degree(v, G) == 1);
                    const auto w = first_source(in_edges(v, G));
                    for (const auto e : make_iterator_range(out_edges(u, G))) {
                        const auto v = target(e, G);
                        addEdge(w, v, G[e]);
                        setModified(v);
                    }
                    modified = true;
                    #ifdef PRINT_DEBUG
                    errs() << "double negation " << u << " -> " << w << "\n";
                    #endif
                    removeVertex(u);
                    continue;
                } else if (LLVM_UNLIKELY(isConstant(negatedTypeId))) {
                    setModified(u);
                    typeId = (negatedTypeId == TypeId::Zeroes) ? TypeId::Ones : TypeId::Zeroes;
                    #ifdef PRINT_DEBUG
                    errs() << "constant negation (" << (int)typeId << ") " << u << "\n";
                    #endif
                    setType(u, typeId);
                    modified = true;
                    goto identity_or_annulment_check;
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
                    modified = true;
                }
            }

            // check whether this vertex is a duplicate
            const auto f = V.insert(u);
            if (LLVM_UNLIKELY(!f.second)) {
                remapVertex(u, *f.first);
            }
        }

        return modified;
    }

    // A XOR NOT(A XOR B) = NOT B

    // A AND NOT(A AND B) = A AND NOT B

    // A AND NOT(A OR B) = 0

    // A OR NOT(A AND B) = 1

    // A OR NOT(A OR B) = A OR NOT B

    // *  (ABSORBTION)       A ∨ (A ∧ B) ⇔ A ∧ (A ∨ B) ⇔ A

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAbsorbtionComplementLaws
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyAbsorbtionComplementLaws() {
        bool modified = false;
        for (const Vertex u : ordering) {
            assert (isLive(u));
            assert (hasValidOperandIndicies(u));
            const TypeId typeId = getType(u);
            if (isDistributive(typeId)) {
                assert (in_degree(u, G) > 0);
                Vertex A[in_degree(u, G)];
                unsigned n = 0;
                for (const auto ei : make_iterator_range(in_edges(u, G))) {
                    const auto v = source(ei, G);
                    assert (isLive(v));
                    const auto innerTypeId = getType(v);
                    auto w = v;
                    if (innerTypeId == TypeId::Not) {
                        w = first_source(in_edges(v, G));
                        assert ("G is cyclic!" && (w != v));
                        assert (isLive(w));
                        for (const auto ej : make_iterator_range(in_edges(u, G))) {
                            if (LLVM_UNLIKELY(source(ej, G) == w)) {
                                goto do_complement;
                            }
                        }
                        if (LLVM_UNLIKELY(getType(w) == oppositeTypeId(typeId))) {
                            // Check for implicit De Morgan's + Complement law application, e.g., A ∧ ¬(A ∨ B) ⇔ 0
                            goto check_demorgans_complement;
                        }
                    } else if (innerTypeId == oppositeTypeId(typeId)) {
check_demorgans_complement:
                        for (const auto ej : make_iterator_range(in_edges(w, G))) {
                            for (const auto ek : make_iterator_range(in_edges(u, G))) {
                                if (LLVM_UNLIKELY(source(ej, G) == source(ek, G))) {
                                    if (LLVM_UNLIKELY(innerTypeId == TypeId::Not)) {
                                        goto do_complement;
                                    } else {
                                        A[n++] = v;
                                        goto next_variable;
                                    }
                                }
                            }
                        }
                    }
next_variable:
                    continue;
                }
                if (LLVM_UNLIKELY(n != 0)) {
                    setModified(u);
                    modified = true;
                    for (;;) {
                        const auto v = A[--n];
                        #ifdef PRINT_DEBUG
                        errs() << "absorbing " << v  << "," << u << "\n";
                        #endif
                        assert (edge(v, u, G).second);
                        remove_edge(v, u, G);assert (isLive(u));
                        if (LLVM_UNLIKELY(out_degree(v, G) == 0)) {
                            removeVertex(v);
                        }
                        if (LLVM_LIKELY(n == 0)) {
                            break;
                        }
                    }
                }
            }
            continue;
do_complement:
            // -----------------------------------------------------------------------------------
            const auto complementTypeId = (typeId == TypeId::And) ? TypeId::Zeroes : TypeId::Ones;
            #ifdef PRINT_DEBUG
            errs() << "complement (" << (int)complementTypeId << ") " << u << "\n";
            #endif
            setModified(u);
            setType(u, complementTypeId);
            clear_in_edges(u, G);
            modified = true;
        }
        return modified;
    }

    void print(raw_ostream & out, const Sequence & S) {
        if (S.empty()) {
            return;
        }
        out << Gd[S[0]];
        for (unsigned i = 1; i < S.size(); ++i) {
            out << ',' << Gd[S[i]];
        }
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

        bool modified = false;

        for (;;) {

            assert (std::is_sorted(D.begin(), D.end()));

            // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
            if (LLVM_UNLIKELY(D.empty())) {
                break;
            }

            #ifdef PRINT_DEBUG
            if (modified) {
                errs() << "--------------------------------------------\n";
            }
            #endif

            const auto lowerSet = obtainDistributableClauses(D);

            for (const auto & lower : lowerSet) {
                const auto & outer = std::get<0>(lower);
                const auto upperSet = obtainDistributableSources(std::get<1>(lower));
                for (const auto & upper : upperSet) {

                    const auto & sources = std::get<1>(upper);
                    const auto & inner = std::get<0>(upper);

                    const auto outerTypeId = getType(Gd[outer[0]]);
                    const auto innerTypeId = oppositeTypeId(outerTypeId);

                    // Update G to match the desired change
                    const auto x = makeVertex(outerTypeId);
                    const auto y = makeVertex(innerTypeId);

                    #ifdef PRINT_DEBUG
                    errs() << "distributing {";
                    print(errs(), sources);
                    if (innerTypeId == TypeId::And) {
                        errs() << "} ∧ {";
                    } else {
                        errs() << "} ∨ {";
                    }
                    print(errs(), inner);
                    if (outerTypeId == TypeId::Or) {
                        errs() << "} ∨ {";
                    } else {
                        errs() << "} ∧ {";
                    }
                    print(errs(), outer);
                    errs() << "} -> " << x << ',' << y << '\n';
                    #endif

                    for (const auto i : inner) {
                        const auto u = Gd[i];
                        assert (getType(u) == innerTypeId);
                        for (const Vertex j : outer) {
                            const auto v = Gd[j];
                            assert (getType(v) == outerTypeId);
                            assert (edge(u, v, G).second);
                            remove_edge(i, j, Gd);
                            remove_edge(u, v, G);
                        }
                        addEdge(u, x);
                    }

                    for (const Vertex i : sources) {
                        const auto u = Gd[i];
                        for (const Vertex j : inner) {
                            const auto v = Gd[j];
                            assert (edge(u, v, G).second);
                            remove_edge(i, j, Gd);
                            remove_edge(u, v, G);
                        }

                        addEdge(u, y);
                    }

                    addEdge(x, y);

                    for (const Vertex i : outer) {
                        const auto u = Gd[i];
                        setModified(u);
                        addEdge(y, u);
                    }

                    modified = true;
                }
            }

            for (;;) {
                if (LLVM_UNLIKELY(D.size() == 1)) {
                    D.clear();
                    break;
                }
                std::uniform_int_distribution<> dist(0, D.size() - 1);
                const auto p = D.begin() + dist(R);
                const auto u = *p;
                D.erase(p);
                bool found = false;
                for (auto e : make_iterator_range(in_edges(u, Gd))) {
                    const auto v = source(e, G);
                    if (canDistribute(v, 1)) {
                        auto f = std::lower_bound(D.begin(), D.end(), v);
                        if (f == D.end() || *f != v) {
                            D.insert(f, v);
                            found = true;
                        }
                    }
                }
                clear_in_edges(u, Gd);
                if (found) {
                    break;
                }
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
        assert (D.empty());
        for (const auto u : make_iterator_range(vertices(G))) {
            if (isLive(u)) {
                const auto outerTypeId = getType(u);
                if (isDistributive(outerTypeId)) {
                    const auto innerTypeId = oppositeTypeId(outerTypeId);
                    for (auto ei : make_iterator_range(in_edges(u, G))) {
                        const auto v = source(ei, G);
                        assert (isLive(v));
                        // can we distribute this vertex?
                        if (getType(v) == innerTypeId) {
                            // is it safe to add v to the distribution graph? I.e., do we need to calculate its value anyway?
                            bool safe = true;
                            for (auto ej : make_iterator_range(out_edges(v, G))) {
                                const auto w = target(ej, G);
                                if (isLive(w) && getType(w) != outerTypeId) {
                                    safe = false;
                                    break;
                                }
                            }
                            if (safe) {
                                D.push_back(v);
                            }
                        }
                    }
                    if (D.size() > 1) {
                        std::sort(D.begin(), D.end());
                        D.erase(std::unique(D.begin(), D.end()), D.end());
                        if (LLVM_LIKELY(D.size() > 1)) {
                            const auto du = addDistributionVertex(u);
                            for (const auto v : D) {
                                assert (isLive(v) && getType(v) == innerTypeId);
                                const auto dv = addDistributionVertex(v);
                                add_edge(dv, du, Gd);
                                for (auto ej : make_iterator_range(in_edges(v, G))) {
                                    const auto x = source(ej, G);
                                    assert (isLive(x));
                                    add_edge(addDistributionVertex(x), dv, Gd);
                                }
                            }
                        }
                    }
                    D.clear();
                }
            }
        }

        assert (D.empty());
        for (auto u : make_iterator_range(vertices(Gd))) {
            if (out_degree(u, Gd) == 0) {
                assert (canDistribute(u));
                D.push_back(u);
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief canDistribute
     ** ------------------------------------------------------------------------------------------------------------- */
    bool canDistribute(const DistributionVertex u, const unsigned outDegree = 0) const {
        assert (u < num_vertices(Gd));
        assert (isLive(Gd[u]));        
        if (out_degree(u, Gd) == outDegree && in_degree(u, Gd) > 0) {
            const auto typeId = oppositeTypeId(getType(Gd[u]));            
            unsigned count = 0;
            for (auto e : make_iterator_range(in_edges(u, Gd))) {
                if (getType(Gd[source(e, Gd)]) == typeId) {
                    if (count == 1) {
                        return true;
                    }
                    ++count;
                }
            }            
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief obtainDistributableClauses
     ** ------------------------------------------------------------------------------------------------------------- */
    BicliqueSet obtainDistributableClauses(const Sequence & S) {

        struct OppositeType {
            bool operator()(const DistributionVertex u) {
                return self.getType(self.Gd[u]) == typeId;
            }
            OppositeType(PassContainer * const pc, const DistributionVertex u)
            : self(*pc)
            , typeId(oppositeTypeId(self.getType(self.Gd[u]))) {

            }
        private:
            PassContainer & self;
            const TypeId typeId;
        };

        struct AllUsers {
            bool operator()(const DistributionVertex u) {
                return out_degree(self.Gd[u], self.G) == degree;
            }
            AllUsers(PassContainer * const pc, const DistributionVertex u)
            : self(*pc)
            , degree(out_degree(self.Gd[u], self.G)) {

            }
        private:
            PassContainer & self;
            const size_t    degree;
        };

        // return makeIndependent(enumerateBicliques<OppositeType, AllUsers>(S, Gd, 1, 2), 1);

        return enumerateBicliques<OppositeType, AllUsers>(S, Gd, 1, 2);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief obtainDistributableSources
     ** ------------------------------------------------------------------------------------------------------------- */
    BicliqueSet obtainDistributableSources(const Sequence & S) {
        return makeIndependent(enumerateBicliques<>(S, Gd, 2, 1), 0);
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
                    const auto m = in_degree(u, G);
                    for (const auto w : T) {
                        remove_edge(w, u, G);
                    }
                    T.clear();
                    const auto n = in_degree(u, G);
                    assert (n != 0 && n <= m);
                    if (LLVM_UNLIKELY(n == 1)) {
                        // An associative operation with only one element is always equivalent to the element
                        const auto v = first_source(in_edges(u, G));
                        #ifdef PRINT_DEBUG
                        errs() << "unary associative " << v << " >> " << u << " (tr)\n";
                        #endif
                        for (auto e : make_iterator_range(out_edges(u, G))) {
                            addEdge(v, target(e, G), G[e]);                            
                        }
                        removeVertex(u);
                    } else if (LLVM_UNLIKELY(m != n)) {
                        #ifdef PRINT_DEBUG
                        errs() << "transitive reduction " << u << "\n";
                        #endif
                        setModified(u);
                    }
                }
            }
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

            for (;;) {

                auto factors = makeIndependent(enumerateBicliques<>(groups[i], G, 2, 2), 1);
                if (factors.empty()) {
                    break;
                }

                bool noChanges = true;

                for (auto & factor : factors) {
                    auto & targets = std::get<0>(factor);
                    assert (targets.size() > 1);
                    auto & sources = std::get<1>(factor);
                    assert (sources.size() > 1);

                    // One of the targets may be our replacement vertex.
                    // If so, its in degree will equal |sources|.
                    Vertex x = 0;
                    bool create = true;
                    for (auto j = targets.begin(); j != targets.end(); ) {
                        assert (hasValidOperandIndicies(*j));
                        if (in_degree(*j, G) == sources.size()) {
                            x = *j;
                            j = targets.erase(j);
                            create = false;
                        } else {
                            ++j;
                        }
                    }
                    if (LLVM_UNLIKELY(targets.empty())) {
                        continue;
                    }
                    if (create) {
                        x = makeVertex(op[i]);
                        groups[i].push_back(x);
                        for (auto u : sources) {
                            addEdge(u, x);
                        }
                    }

                    // Clear out the biclique between the source and target vertices.
                    for (auto u : sources) {
                        for (auto v : targets) {
                            assert (edge(u, v, G).second);
                            boost::remove_edge(u, v, G);
                        }
                    }

                    for (auto v : targets) {
                        assert (getType(v) == op[i]);
                        addEdge(x, v);
                        setModified(v);
                        assert(hasValidOperandIndicies(v));
                    }

                    noChanges = false;
                }
                if (LLVM_UNLIKELY(noChanges)) {
                    break;
                }
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief enumerateBicliques
     *
     * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
     * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
     * bipartition A and their adjacencies to be in B.
     ** ------------------------------------------------------------------------------------------------------------- */
    template <typename Graph>
    struct AcceptAny {
        bool operator()(const typename Graph::vertex_descriptor) { return true; }
        AcceptAny(PassContainer * const, const typename Graph::vertex_descriptor) { }
    };

    template <typename AcceptIntoA = AcceptAny<Graph>, typename AcceptIntoB = AcceptAny<Graph>, typename Graph>
    BicliqueSet enumerateBicliques(const Sequence & S, const Graph & G, const unsigned minimumSizeA = 1, const unsigned minimumSizeB = 1) {
        using IntersectionSets = std::set<Sequence>;

        assert (std::is_sorted(S.begin(), S.end()));

        BicliqueSet cliques(0);

        if (S.size() >= minimumSizeA) {

            IntersectionSets B1;
            for (auto u : S) {
                const auto n = in_degree(u, G);
                if (n > 0) {
                    Sequence B;
                    B.reserve(n);
                    AcceptIntoB acceptor(this, u);
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        const auto v = source(e, G);
                        if (acceptor(v)) {
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

            Sequence clique;
            for (auto i = B1.begin(); i != B1.end(); ++i) {
                assert (std::is_sorted(i->begin(), i->end()));
                for (auto j = i; ++j != B1.end(); ) {
                    assert (std::is_sorted(j->begin(), j->end()));
                    std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
                    if (clique.size() >= minimumSizeB) {
                        if (B.count(clique) == 0) {
                            Bi.insert(clique);
                        }
                    }
                    clique.clear();
                }
            }

            IntersectionSets Bk;
            for (;;) {
                if (Bi.empty()) {
                    break;
                }
                B.insert(Bi.begin(), Bi.end());
                for (auto i = B1.begin(); i != B1.end(); ++i) {
                    assert (std::is_sorted(i->begin(), i->end()));
                    for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                        assert (std::is_sorted(j->begin(), j->end()));
                        std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
                        if (clique.size() >= minimumSizeB) {
                            if (B.count(clique) == 0) {
                                Bk.insert(clique);
                            }
                        }
                        clique.clear();
                    }
                }
                Bi.swap(Bk);
                Bk.clear();
            }

            cliques.reserve(B.size());

            Sequence Aj;
            Sequence Ak;
            for (auto && Bi : B) {
                Sequence Ai(S);
                assert (Bi.size() >= minimumSizeB);
                bool valid = true;
                for (const Vertex u : Bi) {
                    assert (std::is_sorted(Ai.begin(), Ai.end()));
                    assert (Ai.size() >= minimumSizeA);
                    Aj.clear();
                    Aj.reserve(out_degree(u, G));
                    AcceptIntoA acceptor(this, u);
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        const auto v = target(e, G);
                        if (acceptor(v)) {
                            Aj.push_back(v);
                        }
                    }
                    if (Aj.size() < minimumSizeA) {
                        valid = false;
                        break;
                    }
                    std::sort(Aj.begin(), Aj.end());
                    assert(std::unique(Aj.begin(), Aj.end()) == Aj.end());
                    if (LLVM_UNLIKELY(Aj.size() < minimumSizeA)) {
                        valid = false;
                        break;
                    }
                    Ak.clear();
                    Ak.reserve(std::min(Ai.size(), Aj.size()));
                    std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
                    if (Ak.size() < minimumSizeA) {
                        valid = false;
                        break;
                    }
                    Ai.swap(Ak);
                }
                if (valid) {
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
     * @brief addDistributionVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    DistributionVertex addDistributionVertex(const Vertex u) {
        assert (u < num_vertices(G));
        const auto f = Md.find(u);
        if (f == Md.end()) {
            #ifndef NDEBUG
            for (auto v : make_iterator_range(vertices(Gd))) {
                assert (Gd[v] != u);
            }
            #endif
            const auto du = add_vertex(u, Gd);
            assert (Gd[du] == u);
            Md.emplace(u, du);
            return du;
        }
        return f->second;
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

    #ifndef NDEBUG
    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief hasValidOperandIndicies
     ** ------------------------------------------------------------------------------------------------------------- */
    bool hasValidOperandIndicies(const Vertex u) {
        if (isLive(u)) {
            const auto n = in_degree(u, G);
            const auto typeId = getType(u);
            if (LLVM_UNLIKELY(n == 0)) {
                if (LLVM_LIKELY(isAssociative(typeId) || isNullary(typeId))) {
                    return true;
                }
                errs() << u << " cannot be nullary " << (int)typeId << "\n";
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
                        errs() << u << " has duplicate operands " << V[i] << "\n";
                        return false;
                    }
                }
            } else if (requiredOperands(typeId) == n) {
                bool used[n] = { false };
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    const auto i = G[e];
                    if (LLVM_UNLIKELY(i >= n)) {
                        errs() << u << " has operand index " << i << " exceeds in degree " << n << "\n";
                        return false;
                    } else if (LLVM_UNLIKELY(used[i])) {
                        errs() << u << " has duplicate operand indicies " << i << "\n";
                        return false;
                    }
                    used[i] = true;
                }
            } else {
                errs() << u << " has " << n << " operands but requires " << requiredOperands(typeId) << "\n";
                return false;
            }
        }
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            if (!isLive(v)) {
                errs() << u << " has dead operand " << v << "\n";
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
        assert (!isImmutable(getType(u)));
        assert (isLive(u));
        setDead(u);        
        clear_out_edges(u, G);
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
        assert(isAssociative(getType(u)) || getType(u) == TypeId::Not);
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

    Sequence ordering;

    DistributionGraph Gd;
    flat_map<Vertex, DistributionVertex> Md;

    Sequence D;

    std::default_random_engine R;


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
