/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_PabloAST_H
#define PE_PabloAST_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/Compiler.h>
#include <llvm/ADT/SetVector.h>
#include <slab_allocator.h>
#include <iterator>
#include <unordered_map>

using namespace llvm;

namespace pablo {

class PabloBlock;

class PMDNode;

class PabloAST {
    friend class PMDNode;
    friend class Statement;
    friend class Var;
    friend class If;
    friend class While;
    friend class PabloBlock;
    friend class SymbolGenerator;
    typedef std::unordered_map<std::string, PMDNode *> PMDNodeMap;
public:
    typedef SlabAllocator<4096> Allocator;
    enum class ClassTypeId : unsigned {
        Advance
        , And
        , Assign
        , Call
        , If
        , Integer
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
    inline void addUser(PabloAST * user) {
        mUsers.insert(user);
    }
    inline void removeUser(PabloAST * user) {
        mUsers.remove(user);
    }
    static Allocator        mAllocator;
private:
    const ClassTypeId       mClassTypeId;
    SetVector<PabloAST *>   mUsers;
    PMDNodeMap *            mMetadataMap;
};

bool equals(const PabloAST * expr1, const PabloAST *expr2);

class StatementList;

class String;

class Statement : public PabloAST {
    friend class StatementList;
    friend class If;
    friend class While;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case PabloAST::ClassTypeId::String:
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

    inline PabloAST * replaceUsesOfWith(PabloAST * from, PabloAST * to) {
        if (from == to) {
            return this;
        }
        for (unsigned i = 0; i != getNumOperands(); ++i) {
            if (getOperand(i) == from) {
                return setOperand(i, to);
            }
        }
        return this;
    }

    PabloAST * getOperand(const unsigned index) const {
        assert (index < getNumOperands());
        return mOperand[index];
    }

    PabloAST * setOperand(const unsigned index, PabloAST * value);

    unsigned getNumOperands() const {
        return mOperand.size();
    }

    void insertBefore(Statement * const statement);
    void insertAfter(Statement * const statement);
    void removeFromParent();
    void replaceWith(Statement * const statement);

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
protected:
    Statement(const ClassTypeId id, std::vector<PabloAST *> && operands, const String * name, PabloBlock * parent)
    : PabloAST(id)
    , mName(name)
    , mNext(nullptr)
    , mPrev(nullptr)
    , mParent(parent)
    , mOperand(std::move(operands))
    {
        for (PabloAST * op : mOperand) {
            op->addUser(this);
        }
    }

    virtual ~Statement() = 0;
protected:
    const String *              mName;
    Statement *                 mNext;
    Statement *                 mPrev;
    PabloBlock *                mParent;
    std::vector<PabloAST *>     mOperand;

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

    Statement * front() const {
        return mFirst;
    }

    Statement * back() const {
        return mLast;
    }

    void setInsertPoint(Statement * const statement);

    void setInsertPoint(StatementList * const list);

    Statement * getInsertPoint() const {
        return mInsertionPoint;
    }

    void insert(Statement * const statement);

    void insertAfterLastOperand(Statement * const statement);

private:

    Statement   * mInsertionPoint;
    Statement   * mFirst;
    Statement   * mLast;    
};

}

#endif // PE_PabloAST_H



