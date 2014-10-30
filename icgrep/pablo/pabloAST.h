/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <slab_allocator.h>
#include <iterator>

using namespace llvm;

namespace pablo {

class PabloBlock;

class PabloAST {
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
    inline static void release_memory() {
        mAllocator.release_memory();
    }
protected:
    inline PabloAST(const ClassTypeId id)
    : mClassTypeId(id)
    {

    }
    static Allocator    mAllocator;
private:
    const ClassTypeId   mClassTypeId;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

class Statement : public PabloAST {
    friend class StatementList;
public:
    Statement(const ClassTypeId id)
    : PabloAST(id)
    , mNext(nullptr)
    , mPrev(nullptr)
    {

    }
    inline void insertBefore(Statement * const statement) {
        assert (statement);
        mNext = statement;
        mPrev = statement->mPrev;
        statement->mPrev = this;
    }
    inline void insertAfter(Statement * const statement) {
        assert (statement);
        mPrev = statement;
        mNext = statement->mNext;
        statement->mNext = this;
    }
private:
    Statement * mNext;
    Statement * mPrev;
};

class StatementList {

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

    const_iterator begin() const {
        return const_iterator(mFirst);
    }

    const_iterator end() const {
        return const_iterator(nullptr);
    }

    const_iterator cbegin() const {
        return const_iterator(mFirst);
    }

    const_iterator cend() const {
        return const_iterator(nullptr);
    }

    void push_back(Statement * const statement);

private:
    Statement * mFirst;
    Statement * mLast;
};

}

#endif // PE_PabloAST_H



