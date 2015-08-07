/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/Compiler.h>
#include <vector>
#include <slab_allocator.h>
#include <iterator>
#include <unordered_map>

using namespace llvm;

namespace pablo {

class PabloBlock;
class String;

class PabloAST {
    friend class Statement;
    friend class Var;
    friend class If;    
    friend class While;
    friend class PabloBlock;
    friend class Prototype;
    friend class PabloFunction;
    friend class SymbolGenerator;
public:

    using Allocator = SlabAllocator<u_int8_t>;
    using VectorAllocator = SlabAllocator<PabloAST *>;
    using Vector = std::vector<PabloAST*, VectorAllocator>;
    using user_iterator = Vector::iterator;
    using const_user_iterator = Vector::const_iterator;

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
        , Not
        , Xor
        , Sel
        // Stream operations
        , Advance
        , ScanThru
        , MatchStar
        // Mod 64 approximate stream operations
        , Mod64Advance
        , Mod64ScanThru
        , Mod64MatchStar
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

    inline Vector & users() {
        return mUsers;
    }

    inline const Vector & users() const {
        return mUsers;
    }

    void replaceAllUsesWith(PabloAST * expr);

    inline Vector::size_type getNumUses() const {
        return mUsers.size();
    }

    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id)
    , mUsers(mVectorAllocator)
    {

    }
    inline void addUser(PabloAST * user) {
        assert (user);
        auto pos = std::lower_bound(mUsers.begin(), mUsers.end(), user);
        if (LLVM_UNLIKELY(pos != mUsers.end() && *pos == user)) {
            return;
        }
        mUsers.insert(pos, user);
    }
    inline void removeUser(PabloAST * user) {
        assert (user);
        if (mUsers.empty()) {
            return;
        }
        auto pos = std::lower_bound(mUsers.begin(), mUsers.end(), user);
        if (LLVM_UNLIKELY(pos == mUsers.end() || *pos != user)) {
            return;
        }
        mUsers.erase(pos);
    }
    virtual ~PabloAST() {
        mUsers.clear();
    }
    static Allocator        mAllocator;
private:
    const ClassTypeId       mClassTypeId;
    Vector                  mUsers;
    static VectorAllocator  mVectorAllocator;
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

    inline void replaceUsesOfWith(const PabloAST * const from, PabloAST * const to) {
        for (unsigned i = 0; i != getNumOperands(); ++i) {
            if (getOperand(i) == from) {
                setOperand(i, to);
            }
        }
    }

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
    Statement(const ClassTypeId id, std::initializer_list<PabloAST *> operands, const String * const name)
    : PabloAST(id)
    , mName(name)
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(nullptr)
    , mOperands(operands.size())
    , mOperand(reinterpret_cast<PabloAST**>(mAllocator.allocate(mOperands * sizeof(PabloAST *)))) {
        unsigned i = 0;
        for (PabloAST * const op : operands) {
            mOperand[i++] = op;
            if (LLVM_LIKELY(op != nullptr)) {
                op->addUser(this);
            }
        }
    }
    inline void setName(const String * const name) {
        mName = name;
    }    
#ifndef NDEBUG
    bool noRecursiveOperand(const PabloAST * const operand);
#endif
protected:    
    const String *              mName;
    Statement *                 mNext;
    Statement *                 mPrev;
    PabloBlock *                mParent;
    const unsigned              mOperands;
    // If we knew prior to construction how many operands were needed, we could
    // eliminate the mOperand pointer and simply use this[1] instead.
    PabloAST **                 mOperand;
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
    , mLast(nullptr)
    {

    }

    StatementList(StatementList && other)
    : mFirst(other.mFirst)
    , mLast(other.mLast)
    {
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
        mInsertionPoint = statement;
    }

    inline Statement * getInsertPoint() const {
        return mInsertionPoint;
    }

    ~StatementList();

private:

    Statement   * mInsertionPoint;
    Statement   * mFirst;
    Statement   * mLast;    
};

}

#endif // PE_PabloAST_H



