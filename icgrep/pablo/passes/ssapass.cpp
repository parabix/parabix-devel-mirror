#include "ssapass.h"

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/branch.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_var.h>
#include <pablo/pe_phi.h>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <llvm/ADT/SmallVector.h>

#include <llvm/Support/raw_ostream.h>
#include <pablo/printer_pablos.h>

using namespace llvm;
using namespace boost;

namespace pablo {

template<typename K, typename V>
using FlatMap = boost::container::flat_map<K, V>;

template<typename V>
using FlatSet = boost::container::flat_set<V>;

using DefinitionMap = FlatMap<PabloAST *, PabloAST *>;
using PhiMap = FlatMap<std::pair<PabloAST *, PabloAST *>, Phi *>;
using ReachingGraph = adjacency_list<vecS, vecS, bidirectionalS, Statement *, DefinitionMap::value_type>;
using Vertex = ReachingGraph::vertex_descriptor;
using InEdge = graph_traits<ReachingGraph>::in_edge_iterator;

/*
 * Y = Y_0
 * X = 0                          ...
 * While Y:                       While phi(Y_0, Y_i):
 *   ...                             X' = phi(0, X'')
 *   If Z:                           If Z:
 *      X = A                           ...
 *   A = Z                           X'' = phi(X', A)
 *   Y = Y_i
 */

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief buildReachingGraph
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex buildReachingGraph(PabloBlock * const block, Vertex entryPoint, ReachingGraph & G, DefinitionMap & D) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            PabloAST * const var = cast<Assign>(stmt)->getVariable();
            PabloAST * value = cast<Assign>(stmt)->getValue();
            if (isa<Var>(value)) {
                auto f = D.find(value);
                assert (f != D.end());
                value = f->second;
            }
            D[var] = value;
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            const auto branchPoint = add_vertex(stmt, G);
            const auto resumePoint = add_vertex(stmt->getNextNode(), G);
            for (auto p : D) {
                add_edge(entryPoint, branchPoint, p, G);
                add_edge(branchPoint, resumePoint, p, G);
            }
            const Vertex endPoint = buildReachingGraph(cast<Branch>(stmt)->getBody(), branchPoint, G, D);
            if (isa<While>(stmt)) {
                for (auto p : D) {
                    add_edge(endPoint, branchPoint, p, G);
                }
            }
            for (auto p : D) {
                add_edge(endPoint, resumePoint, p, G);
            }
            entryPoint = resumePoint;
        }
    }
    return entryPoint;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVariable
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * getVariable(InEdge e, const ReachingGraph & G) {
    return std::get<0>(G[*e]);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDefinition
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * getDefinition(InEdge e, const ReachingGraph & G, DefinitionMap & D) {
    PabloAST * def = std::get<1>(G[*e]);
    if (LLVM_UNLIKELY(isa<Var>(def))) {
        const auto f = D.find(def);
        if (f != D.end()) {
            def = f->second;
        }
    }
    return def;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generatePhiNodeDefinitions
 ** ------------------------------------------------------------------------------------------------------------- */
void generatePhiNodeDefinitions(const Vertex v, PabloBlock * const block, const ReachingGraph & G, DefinitionMap & D, PhiMap & P) {
    FlatSet<PabloAST *> S;
    InEdge ei, end;
    std::vector<bool> processed(in_degree(v, G), false);
    unsigned i = 0;
    for (std::tie(ei, end) = in_edges(v, G); ei != end; ++ei, ++i) {
        if (processed[i]) {
            continue;
        }
        PabloAST * const var = getVariable(ei, G);
        S.insert(getDefinition(ei, G, D));
        unsigned j = i + 1;
        for (auto ej = ei + 1; ej != end; ++ej, ++j) {
            if (var == getVariable(ej, G)) {
                S.insert(getDefinition(ej, G, D));
            }
        }
        if (S.size() > 1) {
            assert (S.size() == 2);
            auto si = S.begin();
            auto def = std::make_pair(*si, *(si + 1));
            auto f = P.find(def);
            Phi * phi = nullptr;
            if (f == P.end()) {
                phi = block->createPhi(var->getType());
                for (PabloAST * incoming : S) {
                    phi->addIncomingValue(incoming);
                }
                P.emplace(def, phi);
            } else {
                phi = f->second;
            }
            D[var] = phi;
        }
        S.clear();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generatePhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex generatePhiNodes(PabloBlock * const block, Vertex entryPoint, ReachingGraph & G, DefinitionMap & D, PhiMap & P) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            Assign * a = cast<Assign>(stmt);
            PabloAST * const var = a->getVariable();
            PabloAST * value = a->getValue();
            if (isa<Var>(value)) {
                auto f = D.find(value);
                assert (f != D.end());
                value = f->second;
                a->setValue(value);
            }
            D[var] = value;
        } else if (LLVM_UNLIKELY(isa<Extract>(stmt))) {
            continue;
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            const auto branchPoint = entryPoint + 1;
            assert (G[branchPoint] == stmt);
            if (isa<While>(stmt)) {
                generatePhiNodeDefinitions(branchPoint, block, G, D, P);
            }
            Branch * br = cast<Branch>(stmt);
            if (isa<Var>(br->getCondition())) {
                auto f = D.find(br->getCondition());
                assert (f != D.end());
                br->setCondition(f->second);
            }
            const auto resumePoint = entryPoint + 2;
            assert (G[resumePoint] == stmt->getNextNode());

            entryPoint = generatePhiNodes(cast<Branch>(stmt)->getBody(), resumePoint, G, D, P);

            generatePhiNodeDefinitions(resumePoint, block, G, D, P);

        } else {
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                PabloAST * op = stmt->getOperand(i);
                if (isa<Var>(op)) {
                    auto f = D.find(op);
                    assert (f != D.end());
                    stmt->setOperand(i, f->second);
                }
            }
        }
    }
    return entryPoint;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief toSSAForm
 ** ------------------------------------------------------------------------------------------------------------- */
inline void toSSAForm(PabloKernel * const kernel) {
    ReachingGraph G;
    DefinitionMap D;
    PabloBlock * entryBlock = kernel->getEntryBlock();
    const auto entryPoint = add_vertex(entryBlock->front(), G);
    buildReachingGraph(entryBlock, entryPoint, G, D);
    D.clear();
    PhiMap P;
    generatePhiNodes(entryBlock, entryPoint, G, D, P);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
bool SSAPass::transform(PabloKernel * const kernel) {
    toSSAForm(kernel);
    return false;
}


}
