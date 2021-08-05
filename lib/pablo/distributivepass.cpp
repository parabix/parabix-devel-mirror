#include <pablo/distributivepass.h>

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

#include <pablo/pablo_simplifier.hpp>
#include <pablo/pabloverifier.hpp>

#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/function_output_iterator.hpp>
#include <boost/functional/hash.hpp>
#include <util/extended_boost_graph_containers.h>

#include <set>
#include <unordered_set>

#include <boost/graph/strong_components.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/printer_pablos.h>


using namespace boost;
using namespace boost::container;
using namespace llvm;

namespace pablo {

enum class State {
    Dead
    , Live
    , Modified
};

using TypeId = PabloAST::ClassTypeId;

using UsageTime = size_t;

using VertexData = std::tuple<PabloAST *, TypeId, State, UsageTime>;

using OperandIndex = unsigned;

using Graph = adjacency_list<svecS, vecS, bidirectionalS, VertexData, OperandIndex, vecS>;
using Vertex = Graph::vertex_descriptor;
using Edge = Graph::edge_descriptor;
using InEdgeIterator = graph_traits<Graph>::in_edge_iterator;
using DistributionGraph = adjacency_list<setS, vecS, bidirectionalS, Vertex>;
using DistributionVertex = DistributionGraph::vertex_descriptor;

using Sequence = std::vector<Vertex>;
using Biclique = std::pair<Sequence, Sequence>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<Sequence, Sequence, Sequence>;
using DistributionSets = std::vector<DistributionSet>;

using IndependentSetGraph = adjacency_list<vecS, vecS, undirectedS, size_t>;

struct DistributivePassContainer {

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
        readAST(kernel->getEntryScope());
        if (LLVM_LIKELY(simplifyGraph())) {
            rewriteAST(kernel->getEntryScope());
            return true;
        }
        return false;
    }

    DistributivePassContainer()
    : compactedGraph(false)
    , V{0, IdentityHash(G), IdentityComparator(G)} {

    }

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
    size_t rewriteAST(PabloBlock * const entry, size_t count = 0) {

        Statement * stmt = entry->front();

        while (stmt) {

            const Vertex u = findVertex(stmt);

            // There are two reasons to ignore a statement: (1) it's dead or regenerable and thus not of interest,
            // and (2), the value in G strictly dominates the statement. Typically, if two expressions are equivalent,
            // they're in disjoint nested scopes. When this occurs, they're merged into a single vertex in G to avoid
            // redundant computations but must still be written individually in the AST. Sometimes equivalent statements
            // are found during optimization. If one such statement dominates the other, G must refer to the dominating
            // statement to avoid producing an illegal AST.

            if (LLVM_LIKELY(isDead(u) || isRegenerable(getType(u)))) {
                stmt = stmt->eraseFromParent(false);
                continue;
            } else if (LLVM_UNLIKELY(isModified(u) && strictly_dominates(getValue(u), stmt))) {
                stmt = stmt->replaceWith(getValue(u));
                continue;
            }

            assert (isLive(u));
            assert (stmt->getClassTypeId() == getType(u));
            assert (hasValidOperandIndicies(u));
            assert (in_degree(u, G) == stmt->getNumOperands());

            // Suppose we take a subgraph of G that contains every operand we'd have to regenerate to execute this
            // statement? We could apply local optimizations to the subgraph with respect to the last usage time of
            // each non-regenerated value and minimize the pairwise difference in last usage time, taking into account
            // that we can use negations of variables when they've been used more recently.

            // This could take incorporate rematerialization by determining whether it is cheaper / possible to
            // recompute a value with what is already given but the Simplifer Pass could undo these changes if it
            // recognizes a duplicate value in scope.

            // TODO: try setting the insertion point to a dominating position of its known (non-regenerable) users
            // of each operand?

            entry->setInsertPoint(stmt->getPrevNode());
            for (const auto e : make_iterator_range(in_edges(u, G))) {
                const auto v = source(e, G);
                stmt->setOperand(G[e], regenerateIfNecessary(stmt, entry, v, count));
                setLastUsageTime(v, ++count);
            }
            setValue(u, stmt);
            setLastUsageTime(u, ++count);
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                count = rewriteAST(cast<Branch>(stmt)->getBody(), count);
            }

            stmt = stmt->getNextNode();
        }

        return count;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief regenerateIfNecessary
     *
     * Does vertex (u) have a value and, if so, does value dominate the statement? If not, regenerate it.
     ** ------------------------------------------------------------------------------------------------------------- */
    PabloAST * regenerateIfNecessary(const Statement * const stmt, PabloBlock * const entry, const Vertex u, size_t & count) {
        assert (isLive(u));
        PabloAST * value = getValue(u);
        if (dominates(value, stmt)) {
            assert (isNullary(getType(u)) || getLastUsageTime(u) > 0);
        } else { // regenerate the statement ...

            assert (isRegenerable(getType(u)));

            const auto n = in_degree(u, G);

            if (LLVM_UNLIKELY(n == 0)) {
                const TypeId typeId = getType(u);
                assert (isConstant(typeId));
                if (typeId == TypeId::Zeroes) {
                    value = entry->createZeroes();
                } else {
                    value = entry->createOnes();
                }
            } else {

                for (const auto e : make_iterator_range(in_edges(u, G))) {
                    regenerateIfNecessary(stmt, entry, source(e, G), count);
                }

                if (LLVM_LIKELY(n != 1)) {

                    const TypeId typeId = getType(u);

                    assert (isAssociative(typeId));

                    // Suppose we try to minimize the pairwise difference in last usage time,
                    // taking into account that we can use negations of variables when they've
                    // been used more recently. Take note to update the correct vertex if an
                    // ANDC can be used instead.

                    SmallVector<Vertex, 16> input(n);
                    unsigned i = 0;
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        input[i++] = source(e, G);
                    }

                    std::sort(input.begin(), input.end(), [this](const Vertex v, const Vertex w) {
                        return getLastUsageTime(v) < getLastUsageTime(w);
                    });

                    PabloBuilder builder(entry);
                    value = getValue(input[0]);
                    setLastUsageTime(input[0], ++count);
                    for (unsigned i = 1; i < n; ++i) {
                        PabloAST * const op = getValue(input[i]);
                        setLastUsageTime(input[i], ++count);
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
                } else {
                    const auto v = getNegatedLiteral(u);
                    setLastUsageTime(v, ++count);
                    value = entry->createNot(getValue(v));
                }
            }
            assert (value);
            setUnmodified(u);
            setValue(u, value);
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
        if (compactGraph()) {
            goto repeat;
        }
        if (applyDistributivityLaw()) {
            modified = true;
            goto repeat;
        }
        factorizeGraph();
        return modified;
    }

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

                out << "\"";
                if (!hasValidOperandIndicies(u, false)) {
                    out << " color=red style=bold";
                } else if (!(isImmutable(typeId) || out_degree(u, G) > 0)) {
                    out << " color=orange style=bold";
                } else if (isRegenerable(typeId)) {
                    out << " color=blue";
                    if (state == State::Modified) {
                        out << " style=dashed";
                    }
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

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getReverseTopologicalOrdering
     ** ------------------------------------------------------------------------------------------------------------- */
    void getReverseTopologicalOrdering() {

        struct PrePassInserter {
            PrePassInserter & operator=(const Vertex u) {
                if (LLVM_LIKELY(self.isLive(u))) {
                    assert (self.hasValidOperandIndicies(u));
                    if (LLVM_UNLIKELY(isImmutable(self.getType(u)))) {
                        /* do nothing */
                    } else if (LLVM_LIKELY(out_degree(u, self.G) != 0)) {
                        self.ordering.push_back(u);
                    } else {
                        self.removeVertex(u);
                    }
                }
                return *this;
            }

            PrePassInserter(DistributivePassContainer & pc) : self(pc) { }
            PrePassInserter & operator*() { return *this; }
            PrePassInserter & operator++() { return *this; }
            PrePassInserter & operator++(int) { return *this; }

        public:
            DistributivePassContainer & self;
        };

        ordering.clear();
        ordering.reserve(num_vertices(G));
        topological_sort(G, PrePassInserter(*this));
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief compactGraph
     ** ------------------------------------------------------------------------------------------------------------- */
    bool compactGraph() {

        V.clear();
        compactedGraph = false;

        for (const auto u : boost::adaptors::reverse(ordering)) {

            const auto typeId = getType(u);
            assert (isLive(u));
            assert (!isImmutable(typeId));
            assert (hasValidOperandIndicies(u));
            assert (out_degree(u, G) > 0);

            if (LLVM_UNLIKELY(isConstant(typeId))) {
                if (processConstant(u, typeId)) {
                    continue;
                }
            } else if (isAssociative(typeId)) {
                if (processAssociative(u, typeId)) {
                    continue;
                }
            } else if (typeId == TypeId::Not) {
                if (processNegation(u)) {
                    continue;
                }
            }

            consolidate(u);
        }

        return compactedGraph;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief processAssociative
     ** ------------------------------------------------------------------------------------------------------------- */
    bool processAssociative(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (getType(u) == typeId);
        assert (isAssociative(typeId));

        if (LLVM_UNLIKELY(in_degree(u, G) == 0)) {
            setModified(u);
            setType(u, TypeId::Zeroes);
            return processConstant(u, TypeId::Zeroes);
        } else if (LLVM_UNLIKELY(in_degree(u, G) == 1)) {
            // An associative operation with only one element is always equivalent to the element
            const auto v = first_source(in_edges(u, G));
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                addEdge(v, target(e, G), G[e]);
            }
            removeVertex(u);
            compactedGraph = true;
            return true;
        } else {
            // Take the transitive closure of these arcs to reveal the underlying equations
            if (transitiveClosure(u, typeId)) {
                return true;
            }
            if (LLVM_UNLIKELY(typeId == TypeId::Xor)) {
                canonicalizeXor(u);
            } else { // is distributive
                applyDeMorgans(u, typeId);
                return applyAbsorbtionComplementLaw(u, typeId);
            }
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief transitiveClosure
     ** ------------------------------------------------------------------------------------------------------------- */
    bool transitiveClosure(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (getType(u) == typeId);
        assert (isAssociative(typeId));

        SmallVector<Vertex, 16> removed(out_degree(u, G));
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
            }
        }
        if (LLVM_UNLIKELY(out_degree(u, G) == n)) {
            removeVertex(u);
            compactedGraph = true;
            return true;
        }
        for (unsigned i = 0; i < n; ++i) {
            const auto v = removed[i];
            assert (edge(u, v, G).second);
            remove_edge(u, v, G);
            assert(hasValidOperandIndicies(v));
            compactedGraph = true;
        }
        assert (out_degree(u, G) > 0);
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief canonicalizeXor
     *
     * (A ⊕ ¬B) = (A ⊕ B ⊕ 1) = ¬(A ⊕ B)
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex canonicalizeXor(const Vertex u) {

        assert (isLive(u));
        assert (getType(u) == TypeId::Xor);
        assert (in_degree(u, G) > 1);

        const auto l = in_degree(u, G);
        SmallVector<Vertex, 16> negation(l);
        unsigned n = 0, m = 0;
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            const auto typeId = getType(v);
            if (LLVM_UNLIKELY(typeId == TypeId::Not)) {
                negation[n++] = v;
            } else if (LLVM_UNLIKELY(typeId == TypeId::Ones)) {
                negation[l - ++m] = v;
            }
        }
        if (LLVM_UNLIKELY(n != 0 || m != 0)) {
            for (unsigned i = 0; i < n; ++i) {
                const auto v = negation[i];
                assert (edge(v, u, G).second);
                remove_edge(v, u, G);
                addEdge(getNegatedLiteral(v), u);
            }
            for (unsigned i = 0; i < m; ++i) {
                const auto v = negation[(l - 1) - i];
                assert (edge(v, u, G).second);
                remove_edge(v, u, G);
            }
            setModified(u);
            compactedGraph = true;
            if (((n + m) & 1) != 0) {
                const auto x = makeVertex(TypeId::Not);
                for (const auto e : make_iterator_range(out_edges(u, G))) {
                    add_edge(x, target(e, G), G[e], G);
                }
                clear_out_edges(u, G);
                add_edge(u, x, 0, G);
                assert(hasValidOperandIndicies(u));
                return x;
            }
        }
        return u;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief processNegation
     ** ------------------------------------------------------------------------------------------------------------- */
    bool processNegation(const Vertex u) {

        assert (isLive(u));
        assert ("negation must have one input!" && in_degree(u, G) == 1);
        assert (getType(u) == TypeId::Not);

        const auto v = first_source(in_edges(u, G));
        const auto negatedTypeId = getType(v);
        if (LLVM_UNLIKELY(negatedTypeId == TypeId::Not)) { // ¬¬A = A
            const auto w = getNegatedLiteral(v);
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                addEdge(w, v, G[e]);
                setModified(v);
            }
            removeVertex(u);
            compactedGraph = true;
            return true;
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyDeMorgans
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyDeMorgans(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (in_degree(u, G) > 0);
        assert (getType(u) == typeId);
        assert (isDistributive(typeId));

        SmallVector<Vertex, 16> A(in_degree(u, G));
        unsigned n = 0;
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const auto v = source(e, G);
            if (LLVM_UNLIKELY(getType(v) == TypeId::Not)) {
                if (LLVM_UNLIKELY(isDistributive(getType(getNegatedLiteral(v))))) {
                    A[n++] = v;
                }
            }
        }

        if (LLVM_UNLIKELY(n != 0)) {
            for (unsigned i = 0; i < n; ++i) {
                const auto v = A[i];
                assert (edge(v, u, G).second);
                assert (getType(v) == TypeId::Not);
                remove_edge(v, u, G);
                const auto w = getNegatedLiteral(v);
                auto x = u;
                const auto innerTypeId = oppositeTypeId(getType(w));
                if (innerTypeId != typeId) {
                    x = makeVertex(innerTypeId);
                    add_edge(x, u, 0, G);
                }
                for (const auto e : make_iterator_range(in_edges(w, G))) {
                    addEdge(getNegationOf(source(e, G)), x);
                }
            }
            setModified(u);
            assert(hasValidOperandIndicies(u));
            compactedGraph = true;
            return true;
        }
        return false;

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAbsorbtionComplementLaw
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyAbsorbtionComplementLaw(const Vertex u, const TypeId typeId) {

        assert (isLive(u));
        assert (in_degree(u, G) > 0);
        assert (getType(u) == typeId);
        assert (isDistributive(typeId));

        SmallVector<Vertex, 16> A(in_degree(u, G));
        unsigned n = 0;
        for (const auto ei : make_iterator_range(in_edges(u, G))) {
            const auto v = source(ei, G);
            assert (isLive(v));
            const auto innerTypeId = getType(v);
            if (innerTypeId == TypeId::Not) {
                const auto w = first_source(in_edges(v, G));
                assert (isLive(w));
                for (const auto ej : make_iterator_range(in_edges(u, G))) {
                    if (LLVM_UNLIKELY(source(ej, G) == w)) {
                        const auto complementTypeId = (typeId == TypeId::And) ? TypeId::Zeroes : TypeId::Ones;
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

        if (LLVM_UNLIKELY(n != 0)) {
            setModified(u);
            compactedGraph = true;
            do {
                const auto v = A[--n];
                assert (edge(v, u, G).second);
                remove_edge(v, u, G);
            } while (LLVM_UNLIKELY(n != 0));
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
        SmallVector<Vertex, 16> modification(l);
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
                    modification[l - ++n] = v; // fill in from the end
                } else { // annulment
                    setType(v, typeId);
                    modification[m++] = v;
                }
            } else if (LLVM_UNLIKELY(targetTypeId == TypeId::Not)) {
                setType(u, (typeId == TypeId::Zeroes) ? TypeId::Ones : TypeId::Zeroes);
                modification[m++] = v;
            } else { // check if this is a stream operation and optimize accordingly
                if (LLVM_LIKELY(typeId == TypeId::Zeroes)) {
                    switch (targetTypeId) {
                        case TypeId::Advance:
                        case TypeId::IndexedAdvance:
                        case TypeId::Lookahead:
                        case TypeId::InFile:
                        case TypeId::AtEOF:
                            assert (G[e] == 0);
                            setType(v, TypeId::Zeroes);
                            modification[m++] = v;
                            break;
                        case TypeId::ScanThru:
                        case TypeId::AdvanceThenScanThru:
                        case TypeId::MatchStar:
                            if (G[e] == 0) {
                                setType(v, TypeId::Zeroes);
                                modification[m++] = v;
                            } else {
                                strengthReduction(v);
                            }
                            break;
                        case TypeId::ScanTo:
                        case TypeId::AdvanceThenScanTo:
                            strengthReduction(v);
                        default: break;
                    }
                } else { // if (typeId == TypeId::Ones) {
                    switch (targetTypeId) {
                        case TypeId::ScanThru:
                            if (G[e] == 1) {
                                setType(v, TypeId::Zeroes);
                                modification[m++] = v;
                            }
                            break;
                        case TypeId::MatchStar:
                            if (G[e] == 0) {
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

        compactedGraph = true;

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
        }

        if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
            removeVertex(u);
            return true;
        }

        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief strengthReduction
     ** ------------------------------------------------------------------------------------------------------------- */
    void strengthReduction(const Vertex u) {


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
                const auto y = makeVertex(innerTypeId);
                addEdge(consolidate(x), y);
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
                const auto yy = consolidate(y);
                for (const Vertex i : outer) {
                    const auto u = Gd[i];
                    assert (getType(u) == outerTypeId);
                    setModified(u);
                    addEdge(yy, u);
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
                    SmallVector<Vertex, 16> D(n);
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
        cliques.erase(std::remove_if(cliques.begin(), cliques.end(), [this](Biclique & C){
            const auto & A = std::get<0>(C);
            auto & B = std::get<1>(C);
            B.erase(std::remove_if(B.begin(), B.end(), [this, &A](const DistributionVertex u) {
                return out_degree(Gd[u], G) != A.size();
            }), B.end());
            return B.size() < 2;
        }), cliques.end());

        return makeIndependent(std::move(cliques), 0);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief obtainDistributableSources
     ** ------------------------------------------------------------------------------------------------------------- */
    BicliqueSet obtainDistributableSources(const Sequence & S) {
        return makeIndependent(enumerateBicliques(S, Gd, 2, 1), 0);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief factorizeGraph
     *
     * Factorize the associative operations in the final graph
     ** ------------------------------------------------------------------------------------------------------------- */
    void factorizeGraph() {

        Sequence associative(0);
        associative.reserve(num_vertices(G));

        for (const auto u : make_iterator_range(vertices(G))) {
            if (isLive(u) && isAssociative(getType(u))) {
                associative.push_back(u);
            }
        }

        // Although we risk losing better combinations by greedily taking the larger factorings,
        // choosing only those of some minSizeB or greater first can significantly reduce the
        // running time of this optimization.

        Sequence group[3];

        const TypeId typeCode[3] = { TypeId::And, TypeId::Or, TypeId::Xor };

        for (unsigned i = 0; i < 3; ++i) {
            assert (getFactorGroup(typeCode[i]) == i);
        }

        for (;;) {

            // This should be made smarter. Ideally I'd like to factor sets of variables that are
            // whose sources are computed around the same point in the program.

            const auto factors = makeIndependent(enumerateBicliques(associative, G, 2, 2), 1);

            if (LLVM_UNLIKELY(factors.empty())) {
                break;
            }

            bool unchanged = true;

            for (const auto & factor : factors) {
                const auto sources = std::get<1>(factor);
                assert (sources.size() > 1);
                const auto targets = std::get<0>(factor);
                assert (targets.size() > 1);

                for (unsigned i = 0; i < 3; ++i) {
                    assert (group[i].empty());
                }

                // Group the target sets and check whether any target is the factorization
                Vertex t[3];
                bool create[3] = { true, true, true };
                for (const auto u : targets) {
                    assert (hasValidOperandIndicies(u));
                    const auto k = getFactorGroup(getType(u));
                    if (LLVM_UNLIKELY(in_degree(u, G) == sources.size())) {
                        assert (create[k]);
                        t[k] = u;
                        create[k] = false;
                    } else {
                        group[k].push_back(u);
                    }
                }

                for (unsigned k = 0; k < 3; ++k) {
                    if (LLVM_LIKELY(group[k].size() > (create[k] ? 1 : 0))) {
                        if (LLVM_LIKELY(create[k])) {
                            t[k] = makeVertex(typeCode[k]);
                            for (const auto u : sources) {
                                add_edge(u, t[k], 0, G);
                            }
                            associative.push_back(t[k]);
                            assert (t[k] == consolidate(t[k]));
                        }

                        assert (hasValidOperandIndicies(t[k]));
                        // Remove the biclique between the source and target vertices
                        for (auto u : sources) {
                            for (auto v : group[k]) {
                                assert (getType(v) == typeCode[k]);
                                assert (edge(u, v, G).second);
                                boost::remove_edge(u, v, G);
                            }
                        }
                        // ... and replace it with the factorization
                        for (auto v : group[k]) {
                            assert (getType(v) == typeCode[k]);
                            addEdge(t[k], v);
                            setModified(v);
                            assert(hasValidOperandIndicies(v));
                        }
                        unchanged = false;
                    }
                    group[k].clear();
                }
            }

            if (unchanged) {
                for (const auto & factor : factors) {
                    const auto targets = std::get<0>(factor);
                    for (const auto u : targets) {
                        const auto f = std::lower_bound(associative.begin(), associative.end(), u);
                        if (LLVM_LIKELY(f != associative.end() && *f == u)) {
                            associative.erase(f);
                        }
                    }
                }
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getFactorGroup
     ** ------------------------------------------------------------------------------------------------------------- */
    static unsigned getFactorGroup(const TypeId typeId) {
        switch (typeId) {
            case TypeId::And:
                return 0; break;
            case TypeId::Or:
                return 1; break;
            case TypeId::Xor:
                return 2; break;
            default: llvm_unreachable("impossible");
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
                        assert (std::is_sorted(B.begin(), B.end()));
                        assert (std::unique(B.begin(), B.end()) == B.end());
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
                    assert (std::is_sorted(T.begin(), T.end()));
                    assert (std::unique(T.begin(), T.end()) == T.end());
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
        if (!(isConstant(expr->getClassTypeId()) || isLiteral(expr->getClassTypeId()))) {
            expr->print(errs());
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
        if (LLVM_UNLIKELY(typeId == TypeId::Ternary)) {
            report_fatal_error("Ternary operations not supported by DistributivePass");
        } else if (LLVM_UNLIKELY(typeId == TypeId::Sel)) {
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
            const auto u = makeVertex(typeId, isRegenerable(typeId) ? nullptr : stmt);
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
                SmallVector<Vertex, 16> V(n);
                unsigned i = 0;
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    V[i++] = source(e, G);
                }
                std::sort(V.begin(), V.end());
                for (unsigned i = 1; i != n; ++i) {
                    if (LLVM_UNLIKELY(V[i - 1] == V[i])) {
                        if (report) {
                            errs() << u << " has duplicate operands " << V[i] << "\n";
                        }
                        return false;
                    }
                }
            } else if (requiredOperands(typeId) == n) {
                SmallVector<bool, 16> used(n);
                std::fill_n(used.begin(), n, false);
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
                    errs() << u << " (typeid=" << (unsigned)typeId << ") has " << n << " operands but requires " << requiredOperands(typeId) << "\n";
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
            case TypeId::IndexedAdvance:
                return 3;
            case TypeId::Sel:
                llvm_unreachable("impossible");
            default:
                assert (!isAssociative(typeId) && !isNullary(typeId));
                return 2;
        }
    }

    static bool isNullary(const TypeId typeId) {
        return isConstant(typeId) || isLiteral(typeId);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief consolidate
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex consolidate(const Vertex u) {
        assert (isLive(u));
        assert (hasValidOperandIndicies(u));
        const auto f = V.insert(u);
        if (LLVM_UNLIKELY(f.second)) {
            return u;
        }
        const auto v = *f.first;
        assert (IdentityComparator(G)(u, v));
        replaceVertex(u, v);
        return v;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief replaceVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    void replaceVertex(const Vertex u, const Vertex v) {
        assert (u != v);
        assert (isLive(u) && isLive(v));
        const PabloAST * const expr = getValue(u);
        if (expr) {
            assert (!isRegenerable(getType(u)));
            auto f = M.find(expr);
            assert (f != M.end() && f->second == u);
            f->second = v;
            setValue(v, nullptr);
            setModified(v);
        }
        for (const auto e : make_iterator_range(out_edges(u, G))) {
            addEdge(v, target(e, G), G[e]);
        }
        removeVertex(u);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief removeVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    void removeVertex(const Vertex u) {
        assert (isLive(u));
        setDead(u);
        clear_vertex(u, G);
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

    static TypeId getType(const Vertex u, const Graph & G) {
        assert (u < num_vertices(G));
        return std::get<1>(G[u]);
    }

    TypeId getType(const Vertex u) const {
        return getType(u, G);
    }

    void setType(const Vertex u, const TypeId typeId) {
        assert (u < num_vertices(G));
        std::get<1>(G[u]) = typeId;
    }

    static PabloAST * getValue(const Vertex u, const Graph & G) {
        assert (u < num_vertices(G));
        return std::get<0>(G[u]);
    }

    PabloAST * getValue(const Vertex u) const {
        return getValue(u, G);
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
        setValue(u, nullptr);
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
        switch (typeId) {
            case TypeId::Integer: case TypeId::Var: case TypeId::Extract:
                return true;
            default:
                return false;
        }
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

    static TypeId oppositeTypeId(const TypeId typeId) {
        assert (isDistributive(typeId));
        return (typeId == TypeId::And) ? TypeId::Or : TypeId::And;
    }

    Vertex first_source(const std::pair<InEdgeIterator, InEdgeIterator> & e) const {
        return source(*std::get<0>(e), G);
    }

    Vertex getNegatedLiteral(const Vertex u) const {
        assert (getType(u) == TypeId::Not);
        assert (in_degree(u, G) == 1);
        return first_source(in_edges(u, G));
    }

    Vertex removeNegation(const Vertex u) const {
        return getType(u) == TypeId::Not ? getNegatedLiteral(u) : u;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getNegationOf
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex getNegationOf(const Vertex u) {
        if (getType(u) == TypeId::Not) {
            return getNegatedLiteral(u);
        } else {
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                if (getType(v) == TypeId::Not) {
                    return v;
                }
            }
            const auto v = makeVertex(TypeId::Not);
            add_edge(u, v, 0, G);
            return v;
        }
    }

    struct IdentityHash {
        size_t operator()(const Vertex u) const {
            using value_of = std::underlying_type<TypeId>::type;
            #ifndef NDEBUG
            InEdgeIterator begin, end;
            std::tie(begin, end) = in_edges(u, G);
            assert (std::is_sorted(begin, end, [this](const Edge ei, const Edge ej) {
                return source(ei, G) <= source(ej, G);
            }));
            #endif
            size_t h = 0;
            boost::hash_combine(h, static_cast<value_of>(DistributivePassContainer::getType(u, G)));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                boost::hash_combine(h, source(e, G));
            }
            return h;
        }
        IdentityHash (const Graph & g) : G(g) { }
    private:
        const Graph & G;
    };

    struct IdentityComparator {
        bool operator()(const Vertex u, const Vertex v) const {
            const auto typeId = getType(u, G);
            if (LLVM_LIKELY(typeId == getType(v, G))) {
                const unsigned n = in_degree(u, G);
                if (LLVM_UNLIKELY(n == 0)) {
                    assert (isNullary(typeId) && in_degree(v, G) == 0);
                    return getValue(u, G) == getValue(v, G);
                }
                if (in_degree(v, G) == n) {
                    SmallVector<Vertex, 16> adjA(n);
                    SmallVector<Vertex, 16> adjB(n);
                    auto ei = std::get<0>(in_edges(u, G));
                    auto ej = std::get<0>(in_edges(v, G));
                    // if this is an associative op, order doesn't matter
                    if (isAssociative(typeId)) {
                        for (unsigned i = 0; i < n; ++i, ++ei, ++ej) {
                            adjA[i] = source(*ei, G);
                            adjB[i] = source(*ej, G);
                        }
                        assert(std::is_sorted(adjA.begin(), adjA.end()));
                        assert(std::is_sorted(adjB.begin(), adjB.end()));
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
        }
        IdentityComparator (const Graph & g) : G(g) { }
    private:
        const Graph & G;
    };

    using IdentitySet = std::unordered_set<Vertex, IdentityHash, IdentityComparator>;

private:

    bool compactedGraph;

    Graph G;
    flat_map<const pablo::PabloAST *, Vertex> M;

    IdentitySet V;

    DistributionGraph Gd;
    flat_map<Vertex, DistributionVertex> Md;

    Sequence ordering;
    Sequence distributable;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool DistributivePass::optimize(PabloKernel * const kernel) {
    DistributivePassContainer C;
    C.run(kernel);
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-distributive-pass");
    #endif
    Simplifier::optimize(kernel);
    return true;
}

}
