#include "useanalysis.h"

namespace pablo {

void UseAnalysis::optimize(PabloBlock & block) {
    UseAnalysis analyzer;
    analyzer.traverse(analyzer.mRoot, block.expressions());

}

void UseAnalysis::traverse(const Vertex v, StatementList & statements) {
    for (PabloAST * stmt : statements) {
        const Vertex u = find(stmt);
        add_edge(u, v, mUseDefGraph);
        if (const Assign * assign = dyn_cast<Assign>(stmt)) {
            traverse(u, assign->getExpr());
        }
        if (const Next * next = dyn_cast<Next>(stmt)) {
            traverse(u, next->getExpr());
        }
        else if (If * ifStatement = dyn_cast<If>(stmt)) {
            traverse(u, ifStatement->getCondition());
            traverse(u, ifStatement->getBody());
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            traverse(u, whileStatement->getCondition());
            traverse(u, whileStatement->getBody());
        }
    }
}

void UseAnalysis::traverse(const Vertex v, PabloAST * expr) {
    const Vertex u = find(expr);
    add_edge(u, v, mUseDefGraph);
    if (const And * pablo_and = dyn_cast<const And>(expr)) {
        traverse(u, pablo_and->getExpr1());
        traverse(u, pablo_and->getExpr2());
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(expr)) {
        traverse(u, pablo_or->getExpr1());
        traverse(u, pablo_or->getExpr2());
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(expr)) {
        traverse(u, pablo_sel->getCondition());
        traverse(u, pablo_sel->getTrueExpr());
        traverse(u, pablo_sel->getFalseExpr());
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(expr)) {
        traverse(u, pablo_not->getExpr());
    }
    else if (const Advance * adv = dyn_cast<const Advance>(expr)) {
        traverse(u, adv->getExpr());
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr)) {
        traverse(u, mstar->getMarker());
        traverse(u, mstar->getCharClass());
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr)) {
        traverse(u, sthru->getScanFrom());
        traverse(u, sthru->getScanThru());
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
