/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/compile/memoization.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/ErrorHandling.h>
#include <re/adt/adt.h>

using namespace llvm;

namespace re {

static bool compare(const RE * const lh, const RE * const rh);

static bool lessThan(const Seq * const lh, const Seq * const rh) {
    if (LLVM_LIKELY(lh->size() != rh->size())) {
        return lh->size() < rh->size();
    }
    for (auto i = lh->begin(), j = rh->begin(); i != lh->end(); ++i, ++j) {
        assert (*i && *j);
        if (compare(*i, *j)) {
            return true;
        } else if (compare(*j, *i)) {
            return false;
        }
    }
    return false;
}

static bool lessThan(const Alt * const lh, const Alt * const rh) {
    if (LLVM_LIKELY(lh->size() != rh->size())) {
        return lh->size() < rh->size();
    }
    for (auto i = lh->begin(), j = rh->begin(); i != lh->end(); ++i, ++j) {
        assert (*i && *j);
        if (compare(*i, *j)) {
            return true;
        } else if (compare(*j, *i)) {
            return false;
        }
    }
    return false;
}

static bool lessThan(const Name * const lh, const Name * const rh) {
    if (lh->getType() != rh->getType()) {
        return lh->getType() < rh->getType();
    } else if (lh->hasNamespace() != rh->hasNamespace()) {
        return lh->hasNamespace();
    } else if (lh->hasNamespace() && (lh->getNamespace() != rh->getNamespace())) {
        return lh->getNamespace() < rh->getNamespace();
    } else if (lh->getName() != rh->getName()) {
        return lh->getName() < rh->getName();
    } else if (lh->getDefinition() == nullptr) {
        return rh->getDefinition() != nullptr;
    } else if (rh->getDefinition() == nullptr) {
        return false;
    } else {
        return compare(lh->getDefinition(), rh->getDefinition());
    }
}

static bool lessThan(const Assertion * const lh, const Assertion * const rh) {
    if (lh->getKind() != rh->getKind()) {
        return lh->getKind() < rh->getKind();
    }
    if (lh->getSense() != rh->getSense()) {
        return lh->getSense() < rh->getSense();
    }
    return compare(lh->getAsserted(), rh->getAsserted());
}

static bool lessThan(const Rep * const lh, const Rep * const rh) {
    if (lh->getLB() != rh->getLB()) {
        return lh->getLB() < rh->getLB();
    }
    if (lh->getUB() != rh->getUB()) {
        return lh->getUB() < rh->getUB();
    }
    return compare(lh->getRE(), rh->getRE());
}

static bool lessThan(const Diff * const lh, const Diff * const rh) {
    if (compare(lh->getLH(), rh->getLH())) {
        return true;
    } else if (compare(rh->getLH(), lh->getLH())) {
        return false;
    } else if (compare(lh->getRH(), rh->getRH())) {
        return true;
    } else {
        return !compare(rh->getRH(), lh->getRH());
    }
}

static bool lessThan(const Range * const lh, const Range * const rh) {
    if (compare(lh->getLo(), rh->getLo())) {
        return true;
    } else if (compare(rh->getLo(), lh->getLo())) {
        return false;
    } else if (compare(lh->getHi(), rh->getHi())) {
        return true;
    } else {
        return !compare(rh->getHi(), lh->getHi());
    }
}

static bool lessThan(const Intersect * const lh, const Intersect * const rh) {
    if (compare(lh->getLH(), rh->getLH())) {
        return true;
    } else if (compare(rh->getLH(), lh->getLH())) {
        return false;
    } else if (compare(lh->getRH(), rh->getRH())) {
        return true;
    } else {
        return !compare(rh->getRH(), lh->getRH());
    }
}

static bool lessThan(const Group * const lh, const Group * const rh) {
    if (lh->getMode() != rh->getMode()) {
        return lh->getMode() < rh->getMode();
    }
    if (lh->getSense() != rh->getSense()) {
        return lh->getSense() < rh->getSense();
    }
    return compare(lh->getRE(), rh->getRE());
}

static bool compare(const RE * const lh, const RE * const rh) {
    using Type = RE::ClassTypeId;
    assert (lh && rh);
    const auto typeL = lh->getClassTypeId();
    const auto typeR = rh->getClassTypeId();
    if (LLVM_LIKELY(typeL != typeR)) {
        return typeL < typeR;
    }
    switch (typeL) {
        case Type::Alt:
            return lessThan(cast<Alt>(lh), cast<Alt>(rh));
        case Type::Seq:
            return lessThan(cast<Seq>(lh), cast<Seq>(rh));
        case Type::End: case Type::Start:
            return false;
        case Type::Assertion:
            return lessThan(cast<Assertion>(lh), cast<Assertion>(rh));
        case Type::CC:
            return *cast<CC>(lh) < *cast<CC>(rh);
        case Type::Name:
            return lessThan(cast<Name>(lh), cast<Name>(rh));
        case Type::Group:
            return lessThan(cast<Group>(lh), cast<Group>(rh));
        case Type::Range:
            return lessThan(cast<Range>(lh), cast<Range>(rh));
        case Type::Diff:
            return lessThan(cast<Diff>(lh), cast<Diff>(rh));
        case Type::Intersect:
            return lessThan(cast<Intersect>(lh), cast<Intersect>(rh));
        case Type::Rep:
            return lessThan(cast<Rep>(lh), cast<Rep>(rh));
        default:
            llvm_unreachable("RE object of unknown type given to Memoizer");
            return false;
    }
}

bool MemoizerComparator::operator()(const RE * const lh, const RE * const rh) const {
    return compare(lh, rh);
}

} // namespace re
