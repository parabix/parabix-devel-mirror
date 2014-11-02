#include "pablo_automultiplexing.hpp"
#include <pablo/pe_metadata.h>
#include <pablo/pe_advance.h>

#include <llvm/ADT/SmallVector.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/DenseSet.h>
#include <boost/pool/object_pool.hpp>


using namespace llvm;

namespace pablo {

static void AutoMultiplexing::optimize(PabloBlock & block) {
    // indentifyInputVariables


    // generateVariableConstraints



    // identifySuffixConstraints



}


AutoMultiplexing::AutoMultiplexing()
{

}

struct AdvancedStreamGraph {

    struct Vertex;

    typedef std::pair<Vertex *, bool>                   Edge;
    typedef SmallVector<Edge, 1>                        Edges;
    typedef std::vector<Vertex *>                       Verticies;
    typedef Verticies::iterator                         iterator;
    typedef Verticies::const_iterator                   const_iterator;
    typedef Verticies::size_type                        size_type;
    typedef PMDSet                                   VariableConstraintNode;

    struct Vertex {
        Vertex(size_type index, const Advance * advance, const VariableConstraintNode * constraints)
        : _index(index)
        , _advance(advance)
        , _constraints(constraints)
        , _accept(true)
        {
        }

        inline void setConstraintSet(const VariableConstraintNode * constraints) {
            assert (!_constraints && constraints);
            _constraints = constraints;
        }

        inline const VariableConstraintNode * constraintSet() const {
            return _constraints;
        }

        inline bool hasConstraint(const Vertex * other) const {
            return VariableConstraintGraph::hasConstraint(constraintSet(), other->constraintSet());
        }

        inline void addPrecessor(Vertex * predecessor, bool negated = false) {
            assert (predecessor);
            _dependencies.push_back(std::make_pair(predecessor, negated));
            predecessor->_accept = false;
        }

        Edges & predecessors() {
            return _dependencies;
        }

        const Edges & predecessors() const {
            return _dependencies;
        }

        const Advance * advance() const {
            return _advance;
        }

        bool accept() const {
            return _accept;
        }

        size_type index() const {
            return _index;
        }

        const size_type                 _index;
        const Advance *                 _advance;
        const VariableConstraintNode *  _constraints;
        Edges                           _dependencies;
        bool                            _accept;
    };

    inline Vertex * add(const Advance * advance, const VariableConstraintNode * constraints) {
        Vertex * const v = _pool.construct(_verticies.size(), advance, constraints);
        _map.insert(std::make_pair(advance, v));
        _verticies.push_back(v);
        return v;
    }

    inline Vertex * find(const Advance * advance) const {
        auto itr = _map.find(advance);
        if (itr == _map.end()) {
            return nullptr;
        }
        return itr->second;
    }

    static inline bool hasConstraint(const Vertex * a, const Vertex * b)  {
        return VariableConstraintGraph::hasConstraint(a->constraintSet(), b->constraintSet());
    }

    inline static bool hasCharacterClassConstraint(const Edge & a, const Edge & b, const VariableConstraintGraph & U) {
        if (a.second == b.second) {
            DenseSet<const Advance *> set;
            const auto * CCA = a.first->constraintSet();
            set.insert(CCA->begin(), CCA->end());
            const auto * CCB = b.first->constraintSet();
            set.insert(CCB->begin(), CCB->end());
            if (!a.second) {
                // Neither predecessor is from a negated advanced stream.
                //
                // CC(A) ∩ CC(B) ≠ ∅ -> ∃a ∈ CC(A) : a ∈ CC(B) ∧ ∃b ∈ CC(B) : b ∈ CC(A) -> |CC(A) ∪ CC(B)| < |CC(A)| + |CC(B)|
                return set.size() < (CCA->size() + CCB->size());
            }
            else { // if (a.second && b.second)
                // Both predecessors are from negated advanced streams; ergo by De Morgan's law:
                // _____   _____       _____________
                // CC(A) ∩ CC(B) ≠ ∅ ≡ CC(A) ∪ CC(B) ≠ ∅ -> |CC(A) ∪ CC(B)| ≠ |U|
                return set.size() != U.size();
            }
        }
        else {
            // One (and only one) of the predecessors is from a negated advanced stream; thus:
            // _____
            // CC(A) ∩ CC(B) ≠ ∅ -> ∄a ∈ CC(A) : a ∈ CC(B)
            const auto * CCA = a.first->constraintSet();
            const auto * CCB = b.first->constraintSet();
            if (b.second) { // if B is the negated class, swap them for simplicity
                std::swap(CCA, CCB);
            }
            for (const Advance * a : *CCA) {
                if (CCB->count(a)) {
                    return false;
                }
            }
            return true;
        }
    }

    iterator begin() {
        return _verticies.begin();
    }

    iterator end() {
        return _verticies.end();
    }

    const_iterator begin() const {
        return _verticies.begin();
    }

    const_iterator end() const {
        return _verticies.end();
    }

    bool empty() const {
        return _verticies.empty();
    }

    Verticies::size_type size() const {
        return _verticies.size();
    }

    inline void clear() {
        for (Vertex * v : *this) {
            _pool.destroy(v);
        }
        _map.clear();
        _verticies.clear();
    }

private:

    DenseMap<const Advance *, Vertex *>         _map;
    Verticies                                   _verticies;
    boost::object_pool<Vertex>                  _pool;
};

typedef AdvancedStreamGraph::Vertex AdvanceNode;


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeSuffixConstraints
 * @param bb
 * @return if any multiplexing opportunities exist
 ** ------------------------------------------------------------------------------------------------------------- */
bool AutoMultiplexing::initializeSuffixConstraints(PabloBlock & bb) {

    AdvancedStreamGraph G;
    // scan through the graph and compute the advance suffix constraints ...
    for (PabloAST * inst : bb) {
        if (Advance * advance = dyn_cast<Advance>(inst)) {
            VariableConstraintNode * set = _variableConstraints.find(variable);
            AdvanceNode * node = G.add(&advance, set);
            // If this is an initial advance, we'll find the set directly; otherwise
            // we'll have to search for it. Since we cannot be certain that we'll end
            // up parsing the blocks in sequential order, just buffer them for now.
            if (set == nullptr) {
                // first test for the most common case: we're ANDing a previously advanced
                // stream with some character class.
                if (variable->getOpcode() == Instruction::And) {
                    assert (variable->getNumOperands() == 2);
                    int index = 1;
                    VariableConstraintNode * set = nullptr;
                    for (;;) {
                        set = _variableConstraints.find(dyn_cast<Instruction>(variable->getOperand(index)));
                        if (set) {
                            node->setConstraintSet(set);
                            break;
                        }
                        if (index == 0) {
                            break;
                        }
                        --index;
                    }

                    // If we found a character class constraint set for one of the operands; select the other as our preceeding
                    // (advance) stream; otherwise test to see if the variable is the preceeding stream (i.e., there is no CC)

                    Instruction * preceedingStream = set ? dyn_cast<Instruction>(variable->getOperand(1 - index)) : variable;

                    // is this a negated stream?
                    bool negated = false;
                    while (preceedingStream->getOpcode() == Instruction::Xor) {
                        assert (preceedingStream->getNumOperands() == 2);
                        for (unsigned i = 0; i < 2; ++i) {
                            Value * const operand = preceedingStream->getOperand(i);
                            if (isa<Constant>(operand) && dyn_cast<Constant>(operand)->isAllOnesValue()) {
                                negated = !negated;
                                preceedingStream = dyn_cast<Instruction>(preceedingStream->getOperand(1 - i));
                            }
                        }
                    }

                    // If we reach a PHINode, we'll end up handling this through the advance chains anyway. Ignore it.
                    if (isa<PHINode>(preceedingStream)) {
                        continue;
                    }

                    // did we find an immediate predecessor?
                    AdvanceNode * predecessor = G.find(preceedingStream);

                    if (predecessor == nullptr && preceedingStream->getOpcode() == Instruction::And) {
                        assert (preceedingStream->getNumOperands() == 2);
                        for (unsigned i = 0; i < 2; ++i) {
                            Instruction * const operand = dyn_cast<Instruction>(preceedingStream->getOperand(i));
                            if (operand) {
                                if (operand->getOpcode() == Instruction::Xor) {
                                    continue;
                                }
                                predecessor = G.find(operand);
                                break;
                            }
                        }
                    }
                    if (predecessor) {
                        node->addPrecessor(predecessor, negated);
                        continue;
                    }

                    // if not, we could be dealing with a series of disjunctions ORing together the results
                    // of several preceeding streams.
                    if (preceedingStream->getOpcode() == Instruction::Or) {
                        std::queue<Instruction *> disjunctions;
                        Instruction * disjunction = preceedingStream;
                        bool resolvedAllPredecessors = true;
                        for (;;) {
                            assert (disjunction->getNumOperands() == 2);
                            for (unsigned i = 0; i < 2; ++i) {
                                Instruction * const operand = dyn_cast<Instruction>(disjunction->getOperand(i));
                                if (operand && operand->getOpcode() == Instruction::Or) {
                                    AdvanceNode * predecessor = G.find(operand);
                                    if (predecessor) {
                                        node->addPrecessor(predecessor, negated);
                                    }
                                    else {
                                        disjunctions.push(operand);
                                    }
                                    continue;
                                }
                                resolvedAllPredecessors = false;
                            }
                            if (disjunctions.empty()) {
                                break;
                            }
                            disjunction = disjunctions.front();
                            disjunctions.pop();
                        }
                        if (resolvedAllPredecessors) {
                            continue;
                        }
                    }
                }
                #ifdef DEBUG_METADATA_ANNOTATIONS
                advance.setMetadata("unresolved", MDNode::get(bb.getContext(), &advance));
                #endif
            }
        }
    }

    if (G.size() < 3) {
        return false;
    }

    InstructionVector advances;
    advances.reserve(G.size());
    // Generate the initial suffix constraint graph for these advances.
    for (const AdvanceNode * v : G) {
        _suffixConstraints.add(v->advance());
        advances.push_back(const_cast<Instruction *>(v->advance()));
    }
    // Compute the dependency constraints for this graph.
    const auto M = getDependencyMatrixTC<DependencyMatrix>(bb, advances);
    for (auto i = G.begin(); ++i != G.end(); ) {
        const AdvanceNode * const a = *i;
        for (auto j = G.begin(); j != i; ++j) {
            const AdvanceNode * const b = *j;
            if (hasSuffixConstraint(a, b, M)) {
                _suffixConstraints.addConstraint(a->advance(), b->advance());
            }
        }
    }

    return true;
}


}
