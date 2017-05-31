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
    friend class Variadic;
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

    enum class ClassTypeId : unsigned {
        /** Expressions and Constants **/
        // Constants
        Zeroes
        , Ones
        // Arithmetic expressions
        , Add
        , Subtract
        // Relational expressions
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
        , Phi
        /** Statements **/
        // Boolean operations
        , And
        , Or
        , Xor
        , Not        
        , Sel
        // Stream operations
        , Advance
        , ScanThru
        , AdvanceThenScanThru
        , ScanTo
        , AdvanceThenScanTo
        , Lookahead
        , MatchStar
        , InFile
        , AtEOF
        // Statistics operations
        , Count
        // Variable assignments
        , Assign
        , Extract     
        // Scope blocks
        , If
        , While
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

    inline user_iterator user_begin() {
        return mUsers.begin();
    }

    inline user_iterator user_end() {
        return mUsers.end();
    }

    inline const_user_iterator user_begin() const {
        return mUsers.cbegin();
    }

    inline const_user_iterator user_end() const {
        return mUsers.cend();
    }

    inline Users & users() {
        return mUsers;
    }

    inline const Users & users() const {
        return mUsers;
    }

    void replaceAllUsesWith(PabloAST * const expr);

    inline Users::size_type getNumUses() const {
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
    bool addUser(PabloAST * const user);

    bool removeUser(PabloAST * const user);

    virtual ~PabloAST() = default;

private:
    const ClassTypeId       mClassTypeId;
    llvm::Type *            mType;
    Users                   mUsers;
};

bool equals(const PabloAST * const expr1, const PabloAST * const expr2);

bool dominates(const PabloAST * const expr1, const PabloAST * const expr2);

bool postdominates(const PabloAST * const expr1, const PabloAST * const expr2);

class StatementList;

class String;

class Statement : public PabloAST {
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

    const String & getName() const noexcept {
        return *mName;
    }

    void setName(const String * const name) noexcept;

    inline PabloAST * getOperand(const unsigned index) const {
        assert (index < getNumOperands());
        return mOperand[index];
    }

    void setOperand(const unsigned index, PabloAST * const value);

    inline unsigned getNumOperands() const {
        return mOperands;
    }

    void insertBefore(Statement * const statement);
    void insertAfter(Statement * const statement);
    Statement * removeFromParent();
    Statement * eraseFromParent(const bool recursively = false);
    Statement * replaceWith(PabloAST * const expr, const bool rename = true, const bool recursively = false);

    inline Statement * getNextNode() const {
        return mNext;
    }
    inline Statement * getPrevNode() const {
        return mPrev;
    }
    inline PabloBlock * getParent() const {
        return mParent;
    }
    virtual ~Statement() = default;

protected:

    explicit Statement(const ClassTypeId id, llvm::Type * const type, std::initializer_list<PabloAST *> operands, const String * const name, Allocator & allocator)
    : PabloAST(id, type, allocator)
    , mOperands(operands.size())
    , mOperand(allocator.allocate(mOperands))
    , mNext(nullptr)
    , mPrev(nullptr)
    , mName(name)
    , mParent(nullptr) {
        unsigned i = 0;
        for (PabloAST * const value : operands) {
            assert (value);
            mOperand[i] = value;
            value->addUser(this);
            ++i;
        }
    }

    explicit Statement(const ClassTypeId id, llvm::Type * const type, const unsigned reserved, const String * const name, Allocator & allocator)
    : PabloAST(id, type, allocator)
    , mOperands(0)
    , mOperand(allocator.allocate(reserved))
    , mNext(nullptr)
    , mPrev(nullptr)
    , mName(name)
    , mParent(nullptr) {
        std::memset(mOperand, 0, reserved * sizeof(PabloAST *));
    }

    template<typename iterator>
    explicit Statement(const ClassTypeId id, llvm::Type * const type, iterator begin, iterator end, const String * const name, Allocator & allocator)
    : PabloAST(id, type, allocator)
    , mOperands(std::distance(begin, end))
    , mOperand(allocator.allocate(mOperands))
    , mNext(nullptr)
    , mPrev(nullptr)
    , mName(name)
    , mParent(nullptr) {
        unsigned i = 0;
        for (auto value = begin; value != end; ++value, ++i) {
            assert (*value);
            mOperand[i] = *value;
            (*value)->addUser(this);
        }
    }

protected:    
    unsigned        mOperands;
    PabloAST **     mOperand;
    Statement *     mNext;
    Statement *     mPrev;
    const String *  mName;
    PabloBlock *    mParent;
};

class CarryProducingStatement : public Statement {
public:

    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case PabloAST::ClassTypeId::Advance:
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

    virtual ~CarryProducingStatement() = default;

protected:

    explicit CarryProducingStatement(const ClassTypeId id, llvm::Type * const type, std::initializer_list<PabloAST *> operands, const String * const name, Allocator & allocator)
    : Statement(id, type, operands, name, allocator)
    , mCarryGroup(0) {

    }

    explicit CarryProducingStatement(const ClassTypeId id, llvm::Type * const type, const unsigned reserved, const String * name, Allocator & allocator)
    : Statement(id, type, reserved, name, allocator)
    , mCarryGroup(0) {

    }

    template<typename iterator>
    explicit CarryProducingStatement(const ClassTypeId id, llvm::Type * const type, iterator begin, iterator end, const String * name, Allocator & allocator)
    : Statement(id, type, begin, end, name, allocator)
    , mCarryGroup(0) {

    }

private:

    unsigned mCarryGroup;
};


class Variadic : public Statement {
public:

    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
                return true;
            default: return false;
        }
    }
    static inline bool classof(const Variadic *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    class iterator : public boost::iterator_facade<iterator, PabloAST *, boost::random_access_traversal_tag> {
        friend class Variadic;
        friend class boost::iterator_core_access;
    protected:

        iterator(PabloAST ** pointer) : mCurrent(pointer) { }
        inline void increment() { ++mCurrent; }
        inline void decrement() { --mCurrent; }
        inline void advance(const std::ptrdiff_t n) { mCurrent += n; }
        inline std::ptrdiff_t distance_to(const iterator & other) const { return other.mCurrent - mCurrent; }
        inline PabloAST *& dereference() const { return *mCurrent; }

        inline bool equal(const iterator & other) const { return (mCurrent == other.mCurrent); }

    private:
        PabloAST **        mCurrent;
    };

    using const_iterator = iterator;

    void addOperand(PabloAST * const expr);

    PabloAST * removeOperand(const unsigned index);

    bool deleteOperand(const PabloAST * const expr);

    iterator begin() {
        return iterator(mOperand);
    }

    const_iterator begin() const {
        return iterator(mOperand);
    }

    iterator end() {
        return iterator(mOperand + mOperands);
    }

    const_iterator end() const {
        return iterator(mOperand + mOperands);
    }

    virtual ~Variadic() = default;

protected:
    explicit Variadic(const ClassTypeId id, llvm::Type * const type, std::initializer_list<PabloAST *> operands, const String * const name, Allocator & allocator)
    : Statement(id, type, operands, name, allocator)
    , mCapacity(operands.size())
    , mAllocator(allocator) {

    }
    explicit Variadic(const ClassTypeId id, llvm::Type * const type, const unsigned reserved, const String * name, Allocator & allocator)
    : Statement(id, type, reserved, name, allocator)
    , mCapacity(reserved)
    , mAllocator(allocator) {

    }
    template<typename iterator>
    explicit Variadic(const ClassTypeId id, llvm::Type * const type, iterator begin, iterator end, const String * name, Allocator & allocator)
    : Statement(id, type, begin, end, name, allocator)
    , mCapacity(std::distance(begin, end))
    , mAllocator(allocator) {

    }
private:
    unsigned        mCapacity;
    Allocator &     mAllocator;
};

class StatementList {
    friend class Statement;
    friend class PabloBlock;
public:
    class iterator: public std::iterator<std::forward_iterator_tag, Statement> {
    public:
        iterator(): mCurrent(nullptr) {}

        iterator(Statement* base): mCurrent(base) {}

        iterator(const iterator& other): mCurrent(other.mCurrent) {}

        const iterator& operator=(const iterator& other) {
            mCurrent = other.mCurrent; return other;
        }

        inline iterator& operator++() {
            assert (mCurrent);
            mCurrent = mCurrent->mNext;
            return *this;
        }

        iterator  operator++(int) {
            iterator tmp(*this);
            ++(*this);
            return tmp;
        }

        bool operator==(const iterator& other) const {
            return  mCurrent == other.mCurrent;
        }

        bool operator!=(const iterator& other) const {
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
        const const_iterator& operator=(const const_iterator& other) {mCurrent = other.mCurrent; return other;}

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

        const reverse_iterator& operator=(const reverse_iterator& other) {
            mCurrent = other.mCurrent; return other;
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
        const_reverse_iterator(const const_reverse_iterator& other): mCurrent(other.mCurrent) {}
        const const_reverse_iterator& operator=(const const_reverse_iterator& other) {mCurrent = other.mCurrent; return other;}

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

    bool contains(Statement * const statement);

protected:

    ~StatementList() = default;

private:

    Statement   * mInsertionPoint;
    Statement   * mFirst;
    Statement   * mLast;    
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deleteOperand
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool Variadic::deleteOperand(const PabloAST * const expr) {
    for (unsigned i = 0; i != getNumOperands(); ++i) {
        if (LLVM_UNLIKELY(getOperand(i) == expr)) {
            removeOperand(i);
            return true;
        }
    }
    return false;
}

}

#endif // PE_PabloAST_H
