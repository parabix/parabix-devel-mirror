/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "symbol_table.h"

#include <pablo/builder.hpp>
#include <pablo/pabloAST.h>
#include <pablo/pablo_source_kernel.h>
#include <pablo/ps_assign.h>
#include <pablo/parser/error_text.h>
#include <pablo/parser/pablo_type.h>
#include <pablo/parser/token.h>

namespace pablo {
namespace parse {

SymbolTable::Entry::Entry()
: value(nullptr)
, token(nullptr)
, type(nullptr)
, attr(0)
{}

SymbolTable::Entry::Entry(Entry const & other)
: value(other.value)
, token(other.token)
, type(other.type)
, attr(other.attr)
{}

SymbolTable::Entry::Entry(Entry && other)
: value(other.value)
, token(other.token)
, type(other.type)
, attr(std::move(other.attr))
{}

SymbolTable::Entry::Entry(PabloAST * value, Token * token, std::initializer_list<Attr> attr)
: value(value)
, token(token)
, type(nullptr)
, attr(0)
{
    for (auto const & a : attr) {
        this->attr.set(a);
    }
}

SymbolTable::Entry & SymbolTable::Entry::operator = (Entry const & other) {
    this->value = other.value;
    this->token = other.token;
    this->type = other.type;
    this->attr = other.attr;
    return *this;
}

SymbolTable::Entry & SymbolTable::Entry::operator = (Entry && other) {
    this->value = other.value;
    this->token = other.token;
    this->type = other.type;
    this->attr = std::move(other.attr);
    return *this;
}


PabloAST * SymbolTable::assign(Token * token, PabloAST * value) {
    std::string name = token->getText();
    auto optEntry = find(name);
    if (optEntry) {
        Entry & e = *optEntry;
        e.attr.set(Entry::HAS_VALUE);
        PabloAST * const var = e.value;
        assert (var);
        Assign * const assign = mBuilder->createAssign(var, value);
        return llvm::cast<PabloAST>(assign);
    } else {
        Var * const var = mBuilder->createVar(token->getText(), value);
        Entry entry(var, token, {Entry::HAS_VALUE});
        mEntries.insert({name, std::move(entry)});
        return value;
    }
}


PabloAST * SymbolTable::indexedAssign(Token * token, Token * index, PabloAST * value) {
    std::string name = token->getText();
    auto optEntry = find(name);
    if (!optEntry) {
        mErrorManager->logError(token, errtxt_UseOfUndefinedSymbol(name));
        return nullptr;
    }
    Entry & e = optEntry.value();
    if (!e.attr[Entry::INDEXABLE]) {
        mErrorManager->logError(token, errtxt_NonIndexableSymbol(name));
        mErrorManager->logNote(e.token, errtxt_DefinitionNote(name));
        return nullptr;
    }
    uint64_t idx = 0;
    if (index->getType() != TokenType::INT_LITERAL) {
        if (!e.attr[Entry::DOT_INDEXABLE]) {
            mErrorManager->logError(token, errtxt_VarNotDotIndexable(name));
            return nullptr;
        }
        assert (e.type && llvm::isa<NamedStreamSetType>(e.type));
        auto streamset = llvm::cast<NamedStreamSetType>(e.type);
        bool found = false;
        for (auto const & n : streamset->getStreamNames()) {
            if (n == index->getText()) {
                found = true;
                break;
            }
            idx++;
        }
        if (!found) {
            mErrorManager->logError(index, errtxt_InvalidStreamName(name, index->getText()));
            return nullptr;
        }
    } else {
        idx = index->getValue();
    }
    PabloAST * const var = e.value;
    assert (llvm::isa<Var>(var));
    Assign * const assign = mBuilder->createAssign(mBuilder->createExtract(llvm::cast<Var>(var), (int64_t) idx), value);
    return llvm::cast<PabloAST>(assign);
}


PabloAST * SymbolTable::lookup(Token * identifier) {
    assert (identifier->getType() == TokenType::IDENTIFIER);
    std::string name = identifier->getText();
    auto optEntry = find(name);
    if (!optEntry) {
        mErrorManager->logError(identifier, errtxt_UseOfUndefinedSymbol(name));
        return nullptr;
    }
    return optEntry.value().value;
}


PabloAST * SymbolTable::indexedLookup(Token * identifier, Token * index) {
    assert (identifier->getType() == TokenType::IDENTIFIER);
    std::string name = identifier->getText();
    auto optEntry = find(name);
    if (!optEntry) {
        mErrorManager->logError(identifier, errtxt_UseOfUndefinedSymbol(name));
        return nullptr;
    }
    Entry const & e = optEntry.value();
    if (!e.attr[Entry::INDEXABLE]) {
        mErrorManager->logError(identifier, errtxt_NonIndexableSymbol(name));
        mErrorManager->logNote(e.token, errtxt_DefinitionNote(name));
        return nullptr;
    }
    uint64_t idx = 0;
    if (index->getType() != TokenType::INT_LITERAL) {
        if (!e.attr[Entry::DOT_INDEXABLE]) {
            mErrorManager->logError(identifier, errtxt_VarNotDotIndexable(name));
            return nullptr;
        }
        assert (e.type && llvm::isa<NamedStreamSetType>(e.type));
        auto streamset = llvm::cast<NamedStreamSetType>(e.type);
        bool found = false;
        for (auto const & n : streamset->getStreamNames()) {
            if (n == index->getText()) {
                found = true;
                break;
            }
            idx++;
        }
        if (!found) {
            mErrorManager->logError(index, errtxt_InvalidStreamName(name, index->getText()));
            return nullptr;
        }
    } else {
        idx = index->getValue();
    }
    PabloAST * const var = e.value;
    assert (llvm::isa<Var>(var));
    return mBuilder->createExtract(llvm::cast<Var>(var), idx);
}


void SymbolTable::addInputVar(Token * identifier, PabloType * type, PabloSourceKernel * kernel) {
    if (LLVM_UNLIKELY(llvm::isa<AliasType>(type))) {
        addInputVar(identifier, llvm::cast<AliasType>(type)->getAliasedType(), kernel);
        return;
    }
    std::string name = identifier->getText();
    PabloAST * var = nullptr;
    Entry e;
    if (llvm::isa<ScalarType>(type)) {
        mErrorManager->logFatalError(identifier, "input scalars are not supported in pablo kernels");
    } else if (llvm::isa<StreamType>(type)) {
        var = kernel->getInputStreamVar(name);
        e = Entry(var, identifier, {Entry::INPUT, Entry::HAS_VALUE});
    } else if (llvm::isa<StreamSetType>(type)) {
        var = kernel->getInputStreamVar(name);
        e = Entry(var, identifier, {Entry::INPUT, Entry::HAS_VALUE, Entry::INDEXABLE});
    } else if (llvm::isa<NamedStreamSetType>(type)) {
        var = kernel->getInputStreamVar(name);
        e = Entry(var, identifier, {Entry::INPUT, Entry::HAS_VALUE, Entry::INDEXABLE, Entry::DOT_INDEXABLE});
    } else {
        llvm_unreachable(("invalid input type: " + type->asString(true)).c_str());
    }
    e.type = type;
    assert (e.value != nullptr && e.token != nullptr);
    auto rt = mEntries.insert({name, std::move(e)});
    if (!rt.second) {
        mErrorManager->logFatalError(identifier, errtxt_RedefinitionError(name));
        auto prev = mEntries[name];
        mErrorManager->logNote(prev.token, errtxt_PreviousDefinitionNote());
    }
}


void SymbolTable::addOutputVar(Token * identifier, PabloType * type, PabloSourceKernel * kernel) {
    if (LLVM_UNLIKELY(llvm::isa<AliasType>(type))) {
        addOutputVar(identifier, llvm::cast<AliasType>(type)->getAliasedType(), kernel);
        return;
    }
    std::string name = identifier->getText();
    PabloAST * var = nullptr;
    Entry e;
    if (llvm::isa<ScalarType>(type)) {
        var = kernel->getOutputScalarVar(identifier->getText());
        e = Entry(var, identifier, {Entry::OUTPUT});
    } else if (llvm::isa<StreamType>(type)) {
        var = kernel->getOutputStreamVar(identifier->getText());
        e = Entry(var, identifier, {Entry::OUTPUT});
    } else if (llvm::isa<StreamSetType>(type)) {
        var = kernel->getOutputStreamVar(identifier->getText());
        e = Entry(var, identifier, {Entry::OUTPUT, Entry::INDEXABLE});
    } else if (llvm::isa<NamedStreamSetType>(type)) {
        var = kernel->getOutputStreamVar(name);
        e = Entry(var, identifier, {Entry::OUTPUT, Entry::INDEXABLE, Entry::DOT_INDEXABLE});
    } else {
        llvm_unreachable(("invalid output type: " + type->asString(true)).c_str());
    }
    e.type = type;
    assert (e.value != nullptr && e.token != nullptr);
    auto rt = mEntries.insert({name, std::move(e)});
    if (!rt.second) {
        mErrorManager->logFatalError(identifier, errtxt_RedefinitionError(name));
        auto prev = mEntries[name];
        mErrorManager->logNote(prev.token, errtxt_PreviousDefinitionNote());
    }
}


boost::optional<SymbolTable::Entry> SymbolTable::localFind(std::string const & name) {
    auto it = mEntries.find(name);
    if (it == mEntries.end())
        return boost::none;
    return it->second;
}


boost::optional<SymbolTable::Entry> SymbolTable::higherFind(std::string const & name) {
    if (mParent == nullptr)
        return boost::none;
    return mParent->find(name);
}


boost::optional<SymbolTable::Entry> SymbolTable::find(std::string const & name) {
    auto e = localFind(name);
    if (e)
        return e.value();
    return higherFind(name);
}


SymbolTable::SymbolTable(std::shared_ptr<ErrorManager> errorDelegate, PabloBuilder * pb)
: mErrorManager(std::move(errorDelegate))
, mBuilder(pb)
, mEntries()
, mParent(nullptr)
{
    assert (mErrorManager != nullptr);
    assert (mBuilder != nullptr);
}


SymbolTable::SymbolTable(std::shared_ptr<ErrorManager> errorDelegate, PabloBuilder * pb, SymbolTable * parent)
: mErrorManager(std::move(errorDelegate))
, mBuilder(pb)
, mEntries()
, mParent(parent)
{
    assert (mErrorManager != nullptr);
    assert (mBuilder != nullptr);
}

} // namespace pablo::parse
} // namespace pablo
