/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/Compiler.h>
#include <slab_allocator.h>
#include <iterator>
#include <unordered_map>

using namespace llvm;

namespace pablo {

class PabloBlock;

class PMDNode;

class PabloAST {
    friend class PMDNode;
    typedef std::unordered_map<std::string, PMDNode *> PMDNodeMap;
public:
    typedef SlabAllocator<1024> Allocator;
    enum class ClassTypeId : unsigned {
        Advance
        , And
        , Assign
        , Call
        , If
        , MatchStar
        , Next
        , Not
        , Ones
        , Or
        , ScanThru
        , Sel
        , String
        , Var
        , While
        , Xor
        , Zeroes
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }

    inline void replaceUsesOfWith(PabloAST * from, PabloAST * to) {
        if (from == to) {
            return;
        }
        for (unsigned i = 0, operands = getNumOperands(); i != operands; ++i) {
            if (getOperand(i) == from) {
                setOperand(i, to);
            }
        }
    }

    virtual PabloAST * getOperand(const unsigned index) const = 0;

    virtual unsigned getNumOperands() const = 0;

    virtual void setOperand(const unsigned index, PabloAST * value) = 0;

    void setMetadata(const std::string & name, PMDNode * node);

    PMDNode * getMetadata(const std::string & name);

    inline static void release_memory() {
        mAllocator.release_memory();
    }

    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id)
    , mMetadataMap(nullptr)
    {

    }
    static Allocator    mAllocator;
private:
    const ClassTypeId   mClassTypeId;
    PMDNodeMap *        mMetadataMap;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

class StatementList;

class Statement : public PabloAST {
    friend class StatementList;
    friend class If;
    friend class While;
public:
    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::Next:
            case PabloAST::ClassTypeId::If:
            case PabloAST::ClassTypeId::While:
                return true;
            default:
                return false;
        }
    }
    static inline bool classof(const Statement *) {
        return true;
    }
    static inline bool classof(const void *) {
        return false;
    }

    inline void insertBefore(Statement * const statement);
    inline void insertAfter(Statement * const statement);
    inline void removeFromParent();
    inline Statement * getNextNode() const {
        return mNext;
    }
    inline Statement * getPrevNode() const {
        return mPrev;
    }
    inline StatementList * getParent() const {
        return mParent;
    }
protected:
    Statement(const ClassTypeId id, StatementList * parent)
    : PabloAST(id)
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(parent)
    {

    }
    virtual ~Statement() = 0;
protected:
    Statement * mNext;
    Statement * mPrev;
    StatementList * mParent;
};

class StatementList {
    friend class Statement;
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
        friend class iterator;
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
        friend class iterator;
    };

public:

    StatementList()
    : mFirst(nullptr)
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

    Statement * front() const {
        return mFirst;
    }

    Statement * back() const {
        return mLast;
    }

    void push_back(Statement * const statement);

private:
    Statement * mFirst;
    Statement * mLast;
};

inline void Statement::insertBefore(Statement * const statement) {
    assert (statement);
    assert (statement != this);
    assert (statement->mParent);
    removeFromParent();
    mParent = statement->mParent;
    if (LLVM_UNLIKELY(mParent->mFirst == statement)) {
        mParent->mFirst = this;
    }
    mNext = statement;
    mPrev = statement->mPrev;
    statement->mPrev = this;
    if (LLVM_LIKELY(mPrev != nullptr)) {
        mPrev->mNext = this;
    }
}
inline void Statement::insertAfter(Statement * const statement) {
    assert (statement);
    assert (statement != this);
    assert (statement->mParent);
    removeFromParent();
    mParent = statement->mParent;
    if (LLVM_UNLIKELY(mParent->mLast == statement)) {
        mParent->mLast = this;
    }
    mPrev = statement;
    mNext = statement->mNext;
    statement->mNext = this;
    if (LLVM_LIKELY(mNext != nullptr)) {
        mNext->mPrev = this;
    }
}
inline void Statement::removeFromParent() {
    if (LLVM_LIKELY(mParent != nullptr)) {
        if (LLVM_UNLIKELY(mParent->mFirst == this)) {
            mParent->mFirst = mNext;
        }
        if (LLVM_UNLIKELY(mParent->mLast == this)) {
            mParent->mLast = mPrev;
        }
        if (LLVM_LIKELY(mPrev != nullptr)) {
            mPrev->mNext = mNext;
        }
        if (LLVM_LIKELY(mNext != nullptr)) {
            mNext->mPrev = mPrev;
        }
    }
    mPrev = nullptr;
    mNext = nullptr;
    mParent = nullptr;
}

}

#endif // PE_PabloAST_H



