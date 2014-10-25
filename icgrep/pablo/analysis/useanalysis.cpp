#include "useanalysis.h"
#include <queue>

namespace pablo {

void UseAnalysis::optimize(PabloBlock & block) {
    UseAnalysis analyzer;
    analyzer.gatherUseDefInformation(analyzer.mRoot, block.statements());

}

void UseAnalysis::identifyDeadVariables() {
    std::queue<Vertex> Q;
    // gather up all of the nodes that aren't output assignments and have no users
    const auto vMap = get(vertex_name, mUseDefGraph);
    VertexIterator vi, vi_end;
    for (std::tie(vi, vi_end) = vertices(mUseDefGraph); vi != vi_end; ++vi) {
        const Vertex v = *vi;
        if (out_degree(v) == 0) {
            const PabloAST * n = vMap[v];
            if (isa<Assign>(n) && (cast<Assign>(n)->isOutputAssignment())) {
                continue;
            }
            Q.push(v);
        }
    }
    while (!Q.empty()) {
        const Vertex v = Q.front();
        Q.pop();
        InEdgeIterator ei, ei_end;
        for (std::tie(ei, ei_end) = in_edges(v, mUseDefGraph); ei != ei_end; ++ei) {
            const Vertex u = source(*ei, mUseDefGraph);
            if (out_degree(u, mUseDefGraph) == 1) {
                Q.push(u);
            }
        }
        clear_in_edges(v, mUseDefGraph);
    }
}

void UseAnalysis::gatherUseDefInformation(const Vertex v, StatementList & statements) {
    for (PabloAST * stmt : statements) {
        const Vertex u = find(stmt);
        add_edge(u, v, mUseDefGraph);
        if (const Assign * assign = dyn_cast<Assign>(stmt)) {
            gatherUseDefInformation(u, assign->getExpr());
        }
        if (const Next * next = dyn_cast<Next>(stmt)) {
            gatherUseDefInformation(u, next->getExpr());
        }
        else if (If * ifStatement = dyn_cast<If>(stmt)) {
            gatherUseDefInformation(u, ifStatement->getCondition());
            gatherUseDefInformation(u, ifStatement->getBody());
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            gatherUseDefInformation(u, whileStatement->getCondition());
            gatherUseDefInformation(u, whileStatement->getBody());
        }
    }
}

void UseAnalysis::gatherUseDefInformation(const Vertex v, PabloAST * expr) {
    const Vertex u = find(expr);
    add_edge(u, v, mUseDefGraph);
    if (const And * pablo_and = dyn_cast<const And>(expr)) {
        gatherUseDefInformation(u, pablo_and->getExpr1());
        gatherUseDefInformation(u, pablo_and->getExpr2());
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(expr)) {
        gatherUseDefInformation(u, pablo_or->getExpr1());
        gatherUseDefInformation(u, pablo_or->getExpr2());
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(expr)) {
        gatherUseDefInformation(u, pablo_sel->getCondition());
        gatherUseDefInformation(u, pablo_sel->getTrueExpr());
        gatherUseDefInformation(u, pablo_sel->getFalseExpr());
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(expr)) {
        gatherUseDefInformation(u, pablo_not->getExpr());
    }
    else if (const Advance * adv = dyn_cast<const Advance>(expr)) {
        gatherUseDefInformation(u, adv->getExpr());
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr)) {
        gatherUseDefInformation(u, mstar->getMarker());
        gatherUseDefInformation(u, mstar->getCharClass());
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr)) {
        gatherUseDefInformation(u, sthru->getScanFrom());
        gatherUseDefInformation(u, sthru->getScanThru());
    }
}

inline UseAnalysis::Vertex UseAnalysis::find(const PabloAST * const node) {
    auto f = mUseDefMap.find(node);
    if (f == mUseDefMap.end()) {
        Vertex v = add_vertex(mUseDefGraph, node);
        mUseDefMap.insert(std::make_pair(node, v));
        return v;
    }
    return f->second;
}

UseAnalysis::UseAnalysis()
: mRoot(add_vertex(mUseDefGraph, nullptr))
{

}

}
