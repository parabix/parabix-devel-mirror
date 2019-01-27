/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/Compiler.h>
#include <boost/iterator/iterator_facade.hpp>
#include <util/slab_allocator.h>
#include <type_traits>
#include <vector>
namespace llvm { class Type; }
namespace llvm { class raw_ostream; }
namespace pablo { class PabloBlock; }
namespace pablo { class String; }

namespace pablo {

class PabloAST {
    friend class Statement;
    friend class StatementList;
    friend class Branch;
    friend class PabloBlock;
    friend class SymbolGenerator;
    friend class Count;
    friend class Var;
    friend class Operator;
public:

    using Allocator = SlabAllocator<PabloAST *>;
    using Users = std::vector<PabloAST *, ProxyAllocator<PabloAST *>>;
    using user_iterator = Users::iterator;
    using const_user_iterator = Users::const_iterator;

    static inline bool classof(const PabloAST *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    // NOTE: when adding new statement types, update Statement::getName() to generate
    // a default name for the class.
    enum class ClassTypeId : unsigned {
        /** Expressions and Constants **/
        // Constants
        Zeroes
        , Ones
        // Arithmetic expressions
        , Add
        , Subtract
        // Relational operators
        , LessThan
        , LessThanEquals
        , Equals
        , GreaterThanEquals
        , GreaterThan
        , NotEquals
        // Internal types
        , Var
        , Integer
        , String
        , Block
        , Kernel
        , Extract
        /** Statements **/
        // Boolean operations
        , And
        , Or
        , Xor
        , Not
        // Ternary operations
        , Sel
        , Ternary
        // Stream operations
        , Advance
        , IndexedAdvance
        , ScanThru
        , AdvanceThenScanThru
        , ScanTo
        , AdvanceThenScanTo
        , Lookahead
        , MatchStar
        , InFile
        , AtEOF
        , TerminateAt
        // Statistics operations
        , Count
        // Variable assignments
        , Assign
        // Scope branch statements
        , If
        , While
        // Misc. operations
        , Repeat
        , PackH
        , PackL
    };

    inline ClassTypeId getClassTypeId() const noexcept {
        return mClassTypeId;
    }

    inline llvm::Type * getType() const noexcept {
        return mType;
    }

    inline void setType(llvm::Type * type) noexcept {
        mType = type;
    }

    inline user_iterator user_begin() noexcept {
        return mUsers.begin();
    }

    inline user_iterator user_end() noexcept {
        return mUsers.end();
    }

    inline const_user_iterator user_begin() const noexcept {
        return mUsers.cbegin();
    }

    inline const_user_iterator user_end() const noexcept {
        return mUsers.cend();
    }

    inline Users & users() noexcept {
        return mUsers;
    }

    inline const Users & users() const noexcept {
        return mUsers;
    }

    void replaceAllUsesWith(PabloAST * const expr) noexcept;

    inline Users::size_type getNumUses() const noexcept {
        return mUsers.size();
    }

    void * operator new (std::size_t size, Allocator & allocator) noexcept {
        return allocator.allocate<uint8_t>(size);
    }

//    void operator delete (void * ptr) {
//        mAllocator.deallocate(static_cast<Allocator::value_type *>(ptr));
//    }

    void print(llvm::raw_ostream & O) const;

protected:
    inline PabloAST(const ClassTypeId id, llvm::Type * const type, Allocator & allocator)
    : mClassTypeId(id)
    , mType(type)
    , mUsers(allocator) {

    }
    bool addUser(PabloAST * const user) noexcept;

    bool removeUser(PabloAST * const user) noexcept;

    virtual ~PabloAST() = default;

private:
    const ClassTypeId       mClassTypeId;
    llvm::Type *            mType;
    Users                   mUsers;
};

bool equals(const PabloAST * const expr1, const PabloAST * const expr2) noexcept;

bool dominates(const PabloAST * const expr1, const PabloAST * const expr2) noexcept;

inline bool strictly_dominates(const PabloAST * const expr1, const PabloAST * const expr2) noexcept {
    return (expr1 != expr2) && dominates(expr1, expr2);
}

bool postdominates(const PabloAST * const expr1, const PabloAST * const expr2) noexcept;

inline bool strictly_postdominates(const PabloAST * const expr1, const PabloAST * const expr2) noexcept {
    return (expr1 != expr2) && postdominates(expr1, expr2);
}

class String;

class NamedPabloAST : public PabloAST {
public:
    virtual const String & getName() const = 0;
    void setName(const String * const name);
protected:
    explicit NamedPabloAST(const ClassTypeId id, llvm::Type * const type, const String * const name, Allocator & allocator)
    : PabloAST(id, type, allocator)
    , mName(name) {

    }
protected:
    mutable const String * mName;
};

class StatementList;

class Statement : public NamedPabloAST {
    friend class StatementList;
    friend class If;
    friend class While;
    friend class Simplifier;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return ((unsigned)e->getClassTypeId() >= (unsigned)PabloAST::ClassTypeId::And);
    }
    static inline bool classof(const Statement *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    void replaceUsesOfWith(PabloAST * const from, PabloAST * const to);

    inline PabloAST * getOperand(const unsigned index) const noexcept {
        assert (index < getNumOperands());
        return mOperand[index];
    }

    void setOperand(const unsigned index, PabloAST * const value);

    inline unsigned getNumOperands() const {
        return mOperands;
    }

    void insertBefore(Statement * const statement);
    void insertAfter(Statement * const statement);
    Statement * removeFromParent() noexcept;
    Statement * eraseFromParent(const bool recursively = false) noexcept;
    Statement * replaceWith(PabloAST * const expr, const bool rename = true, const bool recursively = false) noexcept;

    inline Statement * getNextNode() const {
        return mNext;
    }
    inline Statement * getPrevNode() const {
        return mPrev;
    }
    inline PabloBlock * getParent() const {
        return mParent;
    }

    const String & getName() const final;

    virtual ~Statement() = default;

protected:

    explicit Statement(const ClassTypeId id, llvm::Type * const type, std::initializer_list<PabloAST *> operands, const String * const name, Allocator & allocator)
    : NamedPabloAST(id, type, name, allocator)
    , mOperands(operands.size())
    , mOperand(allocator.allocate(mOperands))
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(nullptr) {
        unsigned i = 0;
        for (PabloAST * const value : operands) {
            assert (value);
            mOperand[i] = value;
            value->addUser(this);
            ++i;
        }
    }

protected:
    const unsigned          mOperands;
    PabloAST ** const       mOperand;
    Statement *             mNext;
    Statement *             mPrev;
    PabloBlock *            mParent;
};

class CarryProducingStatement : public Statement {
public:

    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::IndexedAdvance:
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::AdvanceThenScanThru:
            case PabloAST::ClassTypeId::ScanTo:
            case PabloAST::ClassTypeId::AdvanceThenScanTo:
            case PabloAST::ClassTypeId::MatchStar:
                return true;
            default: return false;
        }
    }
    static inline bool classof(const CarryProducingStatement *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    unsigned getCarryGroup() const {
        return mCarryGroup;
    }

    void setCarryGroup(const unsigned carryGroup) {
        mCarryGroup = carryGroup;
    }

    unsigned getCarryWidth() const {
        return mCarryWidth;
    }

    void setCarryWidth(const unsigned carryWidth) {
        mCarryWidth = carryWidth;
    }

    virtual ~CarryProducingStatement() = default;

protected:

    explicit CarryProducingStatement(const ClassTypeId id, llvm::Type * const type, std::initializer_list<PabloAST *> operands, const String * const name, Allocator & allocator)
    : Statement(id, type, operands, name, allocator)
    , mCarryGroup(0)
    , mCarryWidth(0) {

    }

private:

    unsigned mCarryGroup;
    unsigned mCarryWidth;
};

class StatementList {
    friend class Statement;
    friend class PabloBlock;
public:
    class iterator: public std::iterator<std::forward_iterator_tag, Statement> {
    public:
        iterator(): mCurrent(nullptr) {}

        iterator(Statement* base): mCurrent(base) {}

        iterator(const iterator & other): mCurrent(other.mCurrent) {}

        iterator & operator=(const iterator & other) {
            mCurrent = other.mCurrent;
            return *this;
        }

        inline iterator& operator++() {
            assert (mCurrent);
            mCurrent = mCurrent->mNext;
            return *this;
        }

        iterator operator++(int) {
            iterator tmp(*this);
            ++(*this);
            return tmp;
        }

        bool operator==(const iterator & other) const {
            return  mCurrent == other.mCurrent;
        }

        bool operator!=(const iterator & other) const {
            return  mCurrent != other.mCurrent;
        }

        Statement* operator*() {return mCurrent;}
        Statement* operator->(){return mCurrent;}

    private:
        Statement * mCurrent;
        friend class const_iterator;
    };

    class const_iterator: public std::iterator<std::forward_iterator_tag, Statement> {
    public:
        const_iterator(): mCurrent(nullptr) {}
        const_iterator(const Statement* base): mCurrent(base) {}
        const_iterator(const const_iterator& other): mCurrent(other.mCurrent) {}
        const_iterator& operator=(const const_iterator & other) {
            mCurrent = other.mCurrent;
            return *this;
        }

        inline const_iterator& operator++() {
            assert (mCurrent);
            mCurrent = mCurrent->mNext;
            return *this;
        }

        const_iterator  operator++(int) {
            const_iterator tmp(*this);
            ++(*this);
            return tmp;
        }

        bool operator==(const const_iterator & other) const {
            return  mCurrent == other.mCurrent;
        }
        bool operator!=(const const_iterator & other) const {
            return  mCurrent != other.mCurrent;
        }

        const Statement* operator*() {return mCurrent;}
        const Statement* operator->(){return mCurrent;}

    private:
        const Statement * mCurrent;
        friend struct iterator;
    };

    class reverse_iterator: public std::iterator<std::forward_iterator_tag, Statement> {
    public:
        reverse_iterator(): mCurrent(nullptr) {}

        reverse_iterator(Statement* base): mCurrent(base) {}

        reverse_iterator(const reverse_iterator& other): mCurrent(other.mCurrent) {}

        reverse_iterator & operator=(const reverse_iterator & other) {
            mCurrent = other.mCurrent;
            return *this;
        }

        inline reverse_iterator& operator++() {
            assert (mCurrent);
            mCurrent = mCurrent->mPrev;
            return *this;
        }

        reverse_iterator operator++(int) {
            reverse_iterator tmp(*this);
            ++(*this);
            return tmp;
        }

        bool operator==(const reverse_iterator& other) const {
            return  mCurrent == other.mCurrent;
        }

        bool operator!=(const reverse_iterator& other) const {
            return  mCurrent != other.mCurrent;
        }

        Statement* operator*() {return mCurrent;}
        Statement* operator->(){return mCurrent;}

    private:
        Statement * mCurrent;
        friend class const_reverse_iterator;
    };

    class const_reverse_iterator: public std::iterator<std::forward_iterator_tag, Statement> {
    public:
        const_reverse_iterator(): mCurrent(nullptr) {}
        const_reverse_iterator(const Statement* base): mCurrent(base) {}
        const_reverse_iterator(const const_reverse_iterator & other): mCurrent(other.mCurrent) {}

        const_reverse_iterator& operator=(const const_reverse_iterator & other) {
            mCurrent = other.mCurrent;
            return *this;
        }

        inline const_reverse_iterator& operator++() {
            assert (mCurrent);
            mCurrent = mCurrent->mPrev;
            return *this;
        }

        const_reverse_iterator  operator++(int) {
            const_reverse_iterator tmp(*this);
            ++(*this);
            return tmp;
        }

        bool operator==(const const_reverse_iterator & other) const {
            return  mCurrent == other.mCurrent;
        }
        bool operator!=(const const_reverse_iterator & other) const {
            return  mCurrent != other.mCurrent;
        }

        const Statement* operator*() {return mCurrent;}
        const Statement* operator->(){return mCurrent;}

    private:
        const Statement * mCurrent;
        friend struct iterator;
    };

public:

    StatementList()
    : mInsertionPoint(nullptr)
    , mFirst(nullptr)
    , mLast(nullptr) {

    }

    StatementList(StatementList && other)
    : mInsertionPoint(nullptr)
    , mFirst(other.mFirst)
    , mLast(other.mLast) {
        other.mInsertionPoint = nullptr;
        other.mFirst = nullptr;
        other.mLast = nullptr;
    }

    iterator begin() {
        return iterator(mFirst);
    }

    iterator end() {
        return iterator(nullptr);
    }

    reverse_iterator rbegin() {
        return reverse_iterator(mLast);
    }

    reverse_iterator rend() {
        return reverse_iterator(nullptr);
    }

    const_iterator begin() const {
        return const_iterator(mFirst);
    }

    const_iterator end() const {
        return const_iterator(nullptr);
    }

    const_reverse_iterator rbegin() const {
        return const_reverse_iterator(mLast);
    }

    const_reverse_iterator rend() const {
        return const_reverse_iterator(nullptr);
    }

    const_iterator cbegin() const {
        return const_iterator(mFirst);
    }

    const_iterator cend() const {
        return const_iterator(nullptr);
    }

    const_reverse_iterator crbegin() const {
        return const_reverse_iterator(mLast);
    }

    const_reverse_iterator crend() const {
        return const_reverse_iterator(nullptr);
    }

    inline Statement * front() const {
        return mFirst;
    }

    inline Statement * back() const {
        return mLast;
    }

    inline void setInsertPoint(Statement * const statement) {
        assert (statement == nullptr || contains(statement));
        mInsertionPoint = statement;
    }

    inline Statement * getInsertPoint() const {
        return mInsertionPoint;
    }

    bool contains(const Statement * const statement) const noexcept;

protected:

    ~StatementList() = default;

private:

    Statement   * mInsertionPoint;
    Statement   * mFirst;
    Statement   * mLast;
};

}

#endif // PE_PabloAST_H
