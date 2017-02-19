/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SCANTHRU_H
#define PS_SCANTHRU_H

#include <pablo/pabloAST.h>

namespace pablo {

class ScanThru : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::ScanThru;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~ScanThru() {
    }
    PabloAST * getScanFrom() const {
        return getOperand(0);
    }
    PabloAST * getScanThru() const {
        return getOperand(1);
    }
protected:
    ScanThru(PabloAST * from, PabloAST * thru, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::ScanThru, from->getType(), {from, thru}, name, allocator) {

    }
};

class ScanTo : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::ScanTo;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~ScanTo() {
    }
    PabloAST * getScanFrom() const {
        return getOperand(0);
    }
    PabloAST * getScanTo() const {
        return getOperand(1);
    }
protected:
    ScanTo(PabloAST * from, PabloAST * to, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::ScanTo, from->getType(), {from, to}, name, allocator) {

    }
};

class AdvanceThenScanThru : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::AdvanceThenScanThru;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~AdvanceThenScanThru() {
    }
    PabloAST * getScanFrom() const {
        return getOperand(0);
    }
    PabloAST * getScanThru() const {
        return getOperand(1);
    }
protected:
    AdvanceThenScanThru(PabloAST * from, PabloAST * thru, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::AdvanceThenScanThru, from->getType(), {from, thru}, name, allocator) {

    }
};

class AdvanceThenScanTo : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::AdvanceThenScanTo;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~AdvanceThenScanTo() {
    }
    PabloAST * getScanFrom() const {
        return getOperand(0);
    }
    PabloAST * getScanTo() const {
        return getOperand(1);
    }
protected:
    AdvanceThenScanTo(PabloAST * from, PabloAST * to, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::AdvanceThenScanTo, from->getType(), {from, to}, name, allocator) {

    }
};

}

#endif // PS_SCANTHRU_H



