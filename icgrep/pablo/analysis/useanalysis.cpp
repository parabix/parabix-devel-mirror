#include "useanalysis.h"
#include <queue>
#include <iostream>

using namespace boost;

namespace pablo {

bool UseAnalysis::optimize(PabloBlock & block) {
    UseAnalysis analyzer;
    analyzer.gatherUseDefInformation(block.statements());
    analyzer.dce();
    analyzer.cse(block);
    return true;
}

void UseAnalysis::cse(PabloBlock & block) {
    VertexIterator vi, vi_end;
    const auto nodeOf = get(vertex_name, mUseDefGraph);
    for (std::tie(vi, vi_end) = vertices(mUseDefGraph); vi != vi_end; ++vi) {
        const Vertex u = *vi;
        const auto count = out_degree(u, mUseDefGraph);
        if (count > 1) {
            PabloAST * expr = nodeOf[u];
            if (isa<Statement>(expr) || isa<Var>(expr)) {
                continue;
            }
            // create the new nodes
            Assign * assign = block.createAssign("cse", expr);
            Var * var = block.createVar(assign);
            const Vertex s = find(assign);
            const Vertex v = find(var);
            // update the program and graph
            OutEdgeIterator ei, ei_end;
            for (std::tie(ei, ei_end) = out_edges(u, mUseDefGraph); ei != ei_end; ++ei) {
                const Vertex t = target(*ei, mUseDefGraph);
                add_edge(v, t, mUseDefGraph);
                PabloAST * user = nodeOf[t];
                user->replaceUsesOfWith(expr, var);
            }
            clear_out_edges(u, mUseDefGraph);
            add_edge(u, s, mUseDefGraph);
            add_edge(s, v, mUseDefGraph);
            Statement * ip = findInsertionPointFor(u, block);
            if (ip == nullptr) {
                assign->insertBefore(block.statements().front());
            }
            else {
                assign->insertAfter(ip);
            }
        }
    }
}

inline Statement * UseAnalysis::findInsertionPointFor(const Vertex v, PabloBlock & block) {
    // We want to find a predecessor of v that is the last statement in the AST.
    const auto nodeOf = get(vertex_name, mUseDefGraph);
    PredecessorSet predecessors;
    std::queue<Vertex> Q;
    InEdgeIterator ei, ei_end;
    for (std::tie(ei, ei_end) = in_edges(v, mUseDefGraph); ei != ei_end; ++ei) {
        const Vertex u = source(*ei, mUseDefGraph);
        PabloAST * n = nodeOf[u];
        if (isa<Statement>(n)) {
            predecessors.insert(cast<Statement>(n));
        }
        else {
            Q.push(u);
        }
    }

    while (!Q.empty()) {
        const Vertex u = Q.front();
        Q.pop();
        PabloAST * n = nodeOf[u];

        if (isa<Statement>(n)) {
            predecessors.insert(cast<Statement>(n));
        }
        else {
            InEdgeIterator ei, ei_end;
            for (std::tie(ei, ei_end) = in_edges(u, mUseDefGraph); ei != ei_end; ++ei) {
                Q.push(source(*ei, mUseDefGraph));
            }
        }
    }
    if (predecessors.empty()) {
        return nullptr;
    }
    return findLastStatement(predecessors, block.statements());
}

Statement * UseAnalysis::findLastStatement(const PredecessorSet & predecessors, StatementList & statements) {
    for (auto ri = statements.rbegin(); ri != statements.rend(); ++ri) {
        Statement * stmt = *ri;
        if (predecessors.count(stmt)) {
            return stmt;
        }
        else if (isa<If>(stmt)) {
            Statement * innerstmt = findLastStatement(predecessors, cast<If>(stmt)->getBody());
            if (innerstmt) {
                return stmt;
            }
        }
        else if (isa<While>(stmt)) {
            stmt = findLastStatement(predecessors, cast<While>(stmt)->getBody());
            if (stmt) {
                return stmt;
            }
        }
    }
    return nullptr;
}

void UseAnalysis::dce() {
    const auto nodeOf = get(vertex_name, mUseDefGraph);
    std::queue<Vertex> Q;
    // gather up all of the nodes that aren't output assignments and have no users
    VertexIterator vi, vi_end;
    for (std::tie(vi, vi_end) = vertices(mUseDefGraph); vi != vi_end; ++vi) {
        const Vertex v = *vi;
        if (out_degree(v, mUseDefGraph) == 0) {
            PabloAST * n = nodeOf[v];
            if (!isa<Assign>(n) || (cast<Assign>(n)->isOutputAssignment())) {
                continue;
            }
            Q.push(v);
        }
    }
    while (!Q.empty()) {
        const Vertex v = Q.front();
        Q.pop();
        PabloAST * n = nodeOf[v];
        if (isa<Assign>(n)) {
            cast<Assign>(n)->removeFromParent();
        }
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

void UseAnalysis::gatherUseDefInformation(const StatementList & statements) {
    for (const Statement * stmt : statements) {
        const Vertex v = find(stmt);
        if (const Assign * assign = dyn_cast<Assign>(stmt)) {
            gatherUseDefInformation(v, assign->getExpr());
        }
        else if (const If * ifStatement = dyn_cast<If>(stmt)) {
            gatherUseDefInformation(v, ifStatement->getCondition());
            gatherUseDefInformation(v, ifStatement->getBody());
        }
        else if (const While * whileStatement = dyn_cast<While>(stmt)) {
            gatherUseDefInformation(v, whileStatement->getCondition());
            gatherUseDefInformation(v, whileStatement->getBody());
        }
        else if (isa<Next>(stmt)) {
            throw std::runtime_error("Next node is illegal in main block!");
        }
    }
}

void UseAnalysis::gatherUseDefInformation(const Vertex v, const StatementList & statements) {
    for (const Statement * stmt : statements) {
        const Vertex u = find(stmt);
        add_edge(u, v, mUseDefGraph);
        if (const Assign * assign = dyn_cast<Assign>(stmt)) {
            gatherUseDefInformation(u, assign->getExpr());
        }
        else if (const Next * next = dyn_cast<Next>(stmt)) {
            add_edge(find(next->getInitial()), u, mUseDefGraph);
            gatherUseDefInformation(u, next->getExpr());
        }
        else if (const If * ifStatement = dyn_cast<If>(stmt)) {
            gatherUseDefInformation(u, ifStatement->getCondition());
            gatherUseDefInformation(u, ifStatement->getBody());
        }
        else if (const While * whileStatement = dyn_cast<While>(stmt)) {
            gatherUseDefInformation(u, whileStatement->getCondition());
            gatherUseDefInformation(u, whileStatement->getBody());
        }
    }
}

void UseAnalysis::gatherUseDefInformation(const Vertex v, const PabloAST * expr) {
    if (isa<Var>(expr) && cast<Var>(expr)->isExternal()) {
        return;
    }
    const Vertex u = find(expr);
    add_edge(u, v, mUseDefGraph);
    if (const Var * var = dyn_cast<Var>(expr)) {
        gatherUseDefInformation(u, var->getVar());
    }
    else if (const And * pablo_and = dyn_cast<const And>(expr)) {
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
        const Vertex v = add_vertex(mUseDefGraph);
        mUseDefMap.insert(std::make_pair(node, v));
        get(vertex_name, mUseDefGraph)[v] = const_cast<PabloAST*>(node);
        return v;
    }
    return f->second;
}
}
