#ifndef FUNCTION_H
#define FUNCTION_H

#include <pablo/pabloAST.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/symbol_generator.h>

namespace pablo {

class Var;
class Assign;
class PabloBlock;
class String;

class PabloFunction : public PabloAST {
    friend class PabloBlock;
    using ParamAllocator = VectorAllocator::rebind<Var *>::other;
    using Parameters = std::vector<Var *, ParamAllocator>;
    using ResultAllocator = VectorAllocator::rebind<Assign *>::other;
    using Results = std::vector<Assign *, ResultAllocator>;
public:

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Function;
    }

    static inline bool classof(const void *) {
        return false;
    }

    static PabloFunction Create(std::string && name);

    virtual bool operator==(const PabloAST & other) const {
        return &other == this;
    }

    PabloBlock & getEntryBlock() {
        return mEntryBlock;
    }

    const String * getName() const {
        return mName;
    }

    const PabloBlock & getEntryBlock() const {
        return mEntryBlock;
    }

    const Parameters & getParameters() const {
        return mParameters;
    }

    const Results & getResults() const {
        return mResults;
    }

    Var * getParameter(const unsigned index) {
        return mParameters[index];
    }

    const Var * getParameter(const unsigned index) const {
        return mParameters[index];
    }

    void addParameter(Var * value) {
        mParameters.push_back(value); value->addUser(this);
    }

    Assign * getResult(const unsigned index) {
        return mResults[index];
    }

    const Assign * getResult(const unsigned index) const {
        return mResults[index];
    }

    void addResult(Assign * value) {
        mResults.push_back(value); value->addUser(this);
    }

    void setResult(const unsigned index, PabloAST * value) {
        getResult(index)->setExpression(value);
    }

    SymbolGenerator & getSymbolTable() {
        return mSymbolTable;
    }

    virtual ~PabloFunction() { }

protected:
    PabloFunction(std::string && name);
private:
    PabloBlock &        mEntryBlock;
    Parameters          mParameters;
    Results             mResults;
    SymbolGenerator     mSymbolTable;
    String *            mName;
};

inline PabloFunction PabloFunction::Create(std::string && name) {
    return PabloFunction(std::move(name));
}

}

#endif // FUNCTION_H
