#ifndef ADVANCEDEPENDENCYINFO_HPP
#define ADVANCEDEPENDENCYINFO_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>

namespace pablo {






namespace {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief DependencyGraphFactory
 * @param bb basic block to scan
 * @param advances the set of advances to look for
 * @param G The graph to write either the dependency graph or transitive closure thereof to
 ** ------------------------------------------------------------------------------------------------------------- */

template <bool transitive_closure>
struct DependencyGraphFactory {

    // Note: this uses a M x N boolean matrix instead of an adjacency matrix; we're only interested in a small portion (< 11.1%) of
    // the dependency graph but need to accumulate edges correctly. Although an adjacency_list is suitable for our purposes here,
    // it is not the ideal choice. Firstly, because we'll end up marking the same edges many times over, we'd need to use a setS
    // for the edges. Secondly, and more importantly, since we're computing the transitive closure of the dependencies, almost every
    // node in the graph will have an edge to one or more advance instructions. This incurs a sizable memory overhead and a high
    // computation cost to constantly rehash the same vertex indices. Compared to the M x N matrix, it increased the cost by ~6.5x.
    // (The standard M x M adjacency_matrix performed far worse, over ~20x.)

    typedef matrix<bool>                                                Graph;
    typedef unsigned                                                    Vertex;
    typedef std::vector<bool>                                           VisitedSet;
    typedef DenseMap<const Instruction *, Vertex>                       Map;
    typedef PabloMultiplexPass::InstructionVector                       InstructionVector;

    DependencyGraphFactory(const BasicBlock & bb, const InstructionVector & advances, Graph & G)
    : _G(G)
    , _BB(bb)
    , _discovered(bb.size())
    {
        _map.resize(bb.size());
        unsigned count = 0;
        for (const Instruction * advance : advances) {
            _map.insert(std::make_pair(advance, count));
            ++count;
        }

        for (const Instruction & inst : bb) {
            if (PabloMultiplexPass::isBooleanLogicInstruction(inst) && _map.insert(std::make_pair(&inst, count)).second) {
                ++count;
            }
        }

        _G.resize(count, advances.size(), false);
        _G.clear();

        for (const Instruction * inst : advances) {
            const Vertex v = find(inst);
            if (undiscovered(v)) {
                visit(v, PabloMultiplexPass::getAdvancedVariable(inst));
            }
        }

    }

    template<typename GraphType>
    static void write(const Graph & G, const unsigned n, GraphType & out) {
        for (unsigned i = 0; i != n; ++i) {
            for (unsigned j = 0; j != n; ++j) {
                if (G(i, j)) {
                    add_edge(i, j, out);
                }
            }
        }
    }

private:

    void visit(const Vertex v, const Instruction * inst) {
        discover(v);
        for (unsigned i = 0; i != inst->getNumOperands(); ++i) {
            const Instruction * operand = dyn_cast<Instruction>(inst->getOperand(i));
            if (PabloMultiplexPass::isLocalBooleanLogicInstruction(operand, _BB)) {
                Vertex u = find(operand);
                // Is this an advance instruction?
                Instruction * const variable = PabloMultiplexPass::getAdvancedVariable(*operand);
                // add an edge between these vertices if so; it's the only way for an edge to exist in G
                if (variable) {
                    _G(v, u) = true;
                    if (!transitive_closure) {
                        continue;
                    }
                }
                // Since dependencies must be acyclic, if we've already encountered this instruction, we do not need
                // to scan it again. Instead we can just adopt it's dependencies.
                if (undiscovered(u)) {
                    // By visiting the input variable directly whenever possible, we reduce the branching factor by ~50%.
                    visit(u, variable ? variable : operand);
                }
                // INVESTIGATE: can we exploit a SIMD OR here? It's possible these ranges could overlap but that
                // can be determined a priori. It's very likely v > u since at least 8/9 instructions cannot be advances.
                for (unsigned w = 0; w < _G.size2(); ++w) {
                    _G(v, w) |= _G(u, w);
                }
            }
        }
    }

    inline void discover(const Vertex v) {
        _discovered[v] = 1;
    }

    inline bool undiscovered(const Vertex v) {
        return _discovered[v] == 0;
    }

    inline Vertex find(const Instruction * inst) {
        return _map.find(inst)->second;
    }

    Graph &                         _G;
    const BasicBlock &              _BB;
    VisitedSet                      _discovered;
    Map                             _map;
};

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDependencyMatrixTC
 * @param bb basic block to scan
 * @param advances the set of advances to look for
 * @return the transitive closure of the advance dependencies in a given graph type format.
 ** ------------------------------------------------------------------------------------------------------------- */

template <typename GraphType>
GraphType PabloMultiplexPass::getDependencyMatrixTC(const BasicBlock & bb, const InstructionVector & advances) {
    typedef DependencyGraphFactory<true> Factory;
    typedef Factory::Graph Graph;
    Graph G;
    Factory(bb, advances, G);
    GraphType D(advances.size());
    Factory::write<GraphType>(G, advances.size(), D);
    return D;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDependencyGraph
 * @param bb basic block to scan
 * @param advances the set of advances to look for
 * @return a graph of the advance dependencies in a given graph type format.
 ** ------------------------------------------------------------------------------------------------------------- */

template <typename GraphType>
GraphType PabloMultiplexPass::getDependencyGraph(const BasicBlock & bb, const InstructionVector & advances) {
    typedef DependencyGraphFactory<false> Factory;
    typedef Factory::Graph Graph;
    Graph G;
    Factory(bb, advances, G);
    GraphType D(advances.size());
    Factory::write<GraphType>(G, advances.size(), D);
    return D;
}

#endif // ADVANCEDEPENDENCYINFO_HPP
