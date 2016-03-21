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
#include <iterator>
#include <util/slab_allocator.h>
#include <type_traits>
#include <unordered_map>
#include <vector>

using namespace llvm;

namespace pablo {

class PabloBlock;
class String;
class Statement;

class PabloAST {
    friend class Statement;
    friend class Variadic;
    friend class StatementList;
    friend class Var;
    friend class If;    
    friend class While;
    friend class PabloBlock;
    friend class Prototype;
    friend class PabloFunction;
    friend class SymbolGenerator;
public:

    using Allocator = SlabAllocator<u_int8_t>;
    using VectorAllocator = Allocator::rebind<PabloAST *>::other;
    using Users = std::vector<PabloAST *, VectorAllocator>;
    using user_iterator = Users::iterator;
    using const_user_iterator = Users::const_iterator;

    enum class ClassTypeId : unsigned {
        /** Non-statements **/
        // Constants
        Zeroes
        , Ones
        // Internal types
        , Var
        , Integer
        , String
        , Block
        , Function
        , Prototype
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
        , Lookahead
        , MatchStar
        // Statistics operations
        , Count
        // Variable assignments
        , Assign
        , Next
        , Call
        , SetIthBit
        // Scope blocks
        , If
        , While
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
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

    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }

    void operator delete (void * ptr) {
        mAllocator.deallocate(static_cast<Allocator::value_type *>(ptr));
    }

protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id)
    , mUsers(reinterpret_cast<VectorAllocator &>(mAllocator))
    {

    }
    void addUser(PabloAST * const user);
    void removeUser(PabloAST * const user);
    virtual ~PabloAST() {
        mUsers.clear();
    }
    static Allocator        mAllocator;
private:
    const ClassTypeId       mClassTypeId;
    Users                   mUsers;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

class StatementList;

class String;

class Statement : public PabloAST {
    friend class StatementList;
    friend class If;
    friend class While;
    friend class Simplifier;
    friend class PabloBlock;
    template <class ValueType, class ValueList>
    friend void checkEscapedValueList(const Statement *, const PabloAST * const, PabloAST * const, ValueList &);
public:
    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case PabloAST::ClassTypeId::Zeroes:
            case PabloAST::ClassTypeId::Ones:
            case PabloAST::ClassTypeId::Var:
            case PabloAST::ClassTypeId::String:
            case PabloAST::ClassTypeId::Integer:
            case PabloAST::ClassTypeId::Block:
            case PabloAST::ClassTypeId::Function:
            case PabloAST::ClassTypeId::Prototype:
                return false;
            default:
                return true;
        }
    }
    static inline bool classof(const Statement *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    void replaceUsesOfWith(PabloAST * const from, PabloAST * const to);

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

    inline const String * getName() const {
        return mName;
    }
    inline void setName(const String * const name) {
        mName = name;
    }
    inline Statement * getNextNode() const {
        return mNext;
    }
    inline Statement * getPrevNode() const {
        return mPrev;
    }
    inline PabloBlock * getParent() const {
        return mParent;
    }
    virtual ~Statement() {}
protected:
    explicit Statement(const ClassTypeId id, std::initializer_list<PabloAST *> operands, const String * const name)
    : PabloAST(id)
    , mName(name)
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(nullptr)
    , mOperands(operands.size())
    , mOperand(reinterpret_cast<PabloAST**>(mAllocator.allocate(mOperands * sizeof(PabloAST *)))) {
        unsigned i = 0;
        for (PabloAST * const value : operands) {
            assert (value);
            mOperand[i] = value;
            value->addUser(this);
            ++i;
        }
    }
    explicit Statement(const ClassTypeId id, const unsigned reserved, const String * const name)
    : PabloAST(id)
    , mName(name)
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(nullptr)
    , mOperands(0)
    , mOperand(reinterpret_cast<PabloAST**>(mAllocator.allocate(reserved * sizeof(PabloAST *)))) {
        std::memset(mOperand, 0, reserved * sizeof(PabloAST *));
    }
    template<typename iterator>
    explicit Statement(const ClassTypeId id, iterator begin, iterator end, const String * const name)
    : PabloAST(id)
    , mName(name)
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(nullptr)
    , mOperands(std::distance(begin, end))
    , mOperand(reinterpret_cast<PabloAST**>(mAllocator.allocate(mOperands * sizeof(PabloAST *)))) {
        unsigned i = 0;
        for (auto value = begin; value != end; ++value, ++i) {
            assert (*value);
            mOperand[i] = *value;
            (*value)->addUser(this);
        }
    }
private:
    template <class ValueType, class ValueList>
    void checkEscapedValueList(Statement * branch, PabloAST * const from, PabloAST * const to, ValueList & list);
protected:    
    const String *  mName;
    Statement *     mNext;
    Statement *     mPrev;
    PabloBlock *    mParent;
    unsigned        mOperands;
    PabloAST **     mOperand;
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

protected:
    explicit Variadic(const ClassTypeId id, std::initializer_list<PabloAST *> operands, const String * const name)
    : Statement(id, operands, name)
    , mCapacity(operands.size()) {

    }
    explicit Variadic(const ClassTypeId id, const unsigned reserved, String * name)
    : Statement(id, reserved, name)
    , mCapacity(reserved) {

    }
    template<typename iterator>
    explicit Variadic(const ClassTypeId id, iterator begin, iterator end, String * name)
    : Statement(id, begin, end, name)
    , mCapacity(std::distance(begin, end)) {

    }
private:
    unsigned        mCapacity;
};

bool escapes(const Statement * statement);

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
 * @brief addUser
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PabloAST::addUser(PabloAST * const user) {
    assert (user);    
    // Note: for the rare situation that this node is used multiple times by the same statement, duplicates are allowed.
    mUsers.insert(std::lower_bound(mUsers.begin(), mUsers.end(), user), user);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeUser
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PabloAST::removeUser(PabloAST * const user) {
    assert (user);
    const auto pos = std::lower_bound(mUsers.begin(), mUsers.end(), user);
    assert ("Could not find user to remove!" && (pos != mUsers.end() && *pos == user));
    mUsers.erase(pos);
}

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
