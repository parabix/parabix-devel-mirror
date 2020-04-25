#include <re/adt/adt.h>
#include <re/adt/printer_re.h>

using namespace llvm;
namespace re {

typedef uint64_t stateVector_t;

class ccSequenceSearchObject {
public:
    ccSequenceSearchObject(std::vector<CC *> & CC_seq) : 
        mCCseq(CC_seq),
        mInitState(1),
        mFinalState(mInitState<<CC_seq.size()),
        mAnyState(mFinalState-1) {
                if (CC_seq.size() >= 64) llvm::report_fatal_error("CC_seq too long!");
        }

    stateVector_t search_from_state(RE * re, stateVector_t v);
    bool search(RE * re);

    std::vector<CC *> mCCseq;
    stateVector_t mInitState;
    stateVector_t mFinalState;
    stateVector_t mAnyState;
};

stateVector_t ccSequenceSearchObject::search_from_state(RE * re, stateVector_t v) {
    if (const Name * n = dyn_cast<Name>(re)) {
        return search_from_state(n->getDefinition(), v);
    } else if (Capture * c = dyn_cast<Capture>(re)) {
        return search_from_state(c->getCapturedRE(), v);
    } else if (LLVM_UNLIKELY(isa<Reference>(re))) {
        llvm::report_fatal_error("back references not supported in icgrep.");
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        stateVector_t rslt = v;
        for (RE * s : *seq) {
            rslt = search_from_state(s, rslt);
            if (rslt & mFinalState) return rslt; // match found
        }
        return rslt;
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        stateVector_t rslt = mInitState;
        for (RE * a : *alt) {
            rslt |= search_from_state(a, v);
        }
        return rslt;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        const auto lb = rep->getLB();
        const auto ub = rep->getUB();
        const auto rpt = rep->getRE();
        stateVector_t rslt = v;
        for (unsigned i = 0; i < lb; i++) {
            stateVector_t next = search_from_state(rpt, rslt);
            if (next == rslt) return next;  // Further repetitions won't change state.
            rslt = next;
            if (rslt & mFinalState) return rslt;  // match found
      }
        for (unsigned i = lb; i != ub; i++) {
            stateVector_t next = search_from_state(rpt, rslt) | rslt;
            if (next == rslt) return next; // Further repetitions won't change state.
            rslt |= next;
            if (rslt & mFinalState) return rslt;  // match found
        }
    } else if (const Assertion * a = dyn_cast<const Assertion>(re)) {
        if (a->getKind() == Assertion::Kind::LookBehind) {
            stateVector_t next = search_from_state(a->getAsserted(), mAnyState);
            if (a->getSense() == Assertion::Sense::Negative) {
                next = ~next;
            }
            return (v & next) | mInitState;
        } else {
            llvm::report_fatal_error("Unsupported assertion.");
        }
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return (search_from_state(diff->getLH(), v) &~ search_from_state(diff->getRH(), v)) | mInitState;
    } else if (const Intersect * ix = dyn_cast<Intersect>(re)) {
        return search_from_state(ix->getLH(), v) & search_from_state(ix->getRH(), v);
    } else if (isa<Start>(re)) {
        return mInitState;
    } else if (isa<End>(re)) {
        return v & mFinalState;
    } else if (const CC * cc = dyn_cast<CC>(re)) {
        stateVector_t CC_matches = 0;
        for (unsigned i = 0; i < mCCseq.size(); i++) {
            CC_matches |= (cc->intersects(*mCCseq[i])) << i;
        }
        return ((CC_matches & v) << 1) | mInitState;
    } else {
        llvm::report_fatal_error("failed to process " + Printer_RE::PrintRE(re));
    }
}

bool ccSequenceSearchObject::search(RE * re) {
    return (search_from_state(re, mInitState) & mFinalState) > 0;
}

bool CC_Sequence_Search(std::vector<CC *> & CC_seq, RE * re) {
    return ccSequenceSearchObject(CC_seq).search(re);
}
}
