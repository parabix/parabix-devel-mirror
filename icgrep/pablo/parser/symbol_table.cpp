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
#include <pablo/parser/token.h>

namespace pablo {
namespace parse {

SymbolTable::Entry::Entry()
: value(nullptr)
, token(nullptr)
, attr(0)
{}

SymbolTable::Entry::Entry(Entry const & other)
: value(other.value)
, token(other.token)
, attr(other.attr)
{}

SymbolTable::Entry::Entry(Entry && other)
: value(other.value)
, token(other.token)
, attr(std::move(other.attr))
{}

SymbolTable::Entry::Entry(PabloAST * value, Token * token, std::initializer_list<Attr> attr)
: value(value)
, token(token)
, attr(0)
{
    for (auto const & a : attr) {
        this->attr.set(a);
    }
}

SymbolTable::Entry & SymbolTable::Entry::operator = (Entry const & other) {
    this->value = other.value;
    this->token = other.token;
    this->attr = other.attr;
    return *this;
}

SymbolTable::Entry & SymbolTable::Entry::operator = (Entry && other) {
    this->value = other.value;
    this->token = other.token;
    this->attr = std::move(other.attr);
    return *this;
}

PabloAST * SymbolTable::createVar(Token * token, PabloAST * value) {
    std::string name = token->getText();
    auto optEntry = localFind(name);
    if (optEntry) {
        mErrorManager->logError(token, errtxt_RedefinitionError(name));
        mErrorManager->logNote(optEntry.value().token, errtxt_PreviousDefinitionNote());
        return nullptr;
    }
    optEntry = higherFind(name);
    if (optEntry) {
        Entry const & e = optEntry.value();
        if (e.attr[Entry::INPUT] || e.attr[Entry::OUTPUT]) {
            // it is an error to redeclare an I/O variable
            mErrorManager->logError(token, errtxt_RedefineIOVarError());
            return nullptr;
        } else {
            mErrorManager->logWarning(token, errtxt_HidesDeclaration(name));
            mErrorManager->logNote(e.token, errtxt_PreviousDefinitionNote());
        }
    }
    PabloAST * const var = mBuilder->createVar(token->getText(), value);
    Entry entry(var, token, {Entry::MUTABLE, Entry::HAS_VALUE});
    mEntries.insert({name, std::move(entry)});
    return var;
}


PabloAST * SymbolTable::assign(Token * token, PabloAST * value) {
    std::string name = token->getText();
    auto optEntry = localFind(name);
    if (!optEntry) {
        optEntry = higherFind(name);
        if (optEntry && !optEntry.value().attr[Entry::MUTABLE]) {
            // Found an entry in a higher symbol table, but it was not a mutable
            // symbol. In this case, we don't want to overwrite the symbol so
            // treat it like we didn't find one at all.
            optEntry = boost::none;
        }
    }
    if (optEntry) {
        Entry & e = *optEntry;
        if (e.attr[Entry::MUTABLE]) {
            PabloAST * const var = e.value;
            assert (var);
            Assign * const assign = mBuilder->createAssign(var, value);
            e.attr.set(Entry::USED_MUTABLE);
            return llvm::cast<PabloAST>(assign);
        } else {
            // must be in local scope
            mEntries[name] = Entry(value, token, {Entry::HAS_VALUE});
            return value;
        }
    } else {
        Entry entry(value, token, {Entry::HAS_VALUE});
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
    if (!e.attr[Entry::INDEXABLE] || !e.attr[Entry::MUTABLE]) {
        mErrorManager->logError(token, errtxt_NonIndexableSymbol(name));
        mErrorManager->logNote(e.token, errtxt_DefinitionNote(name));
        return nullptr;
    }
    if (index->getType() != TokenType::INT_LITERAL) {
        mErrorManager->logError(index, "non-integer indexing is not supported at this time");
        return nullptr;
    }
    uint64_t idx = index->getValue();
    PabloAST * const var = e.value;
    assert (llvm::isa<Var>(var));
    Assign * const assign = mBuilder->createAssign(mBuilder->createExtract(llvm::cast<Var>(var), (int64_t) idx), value);
    e.attr.set(Entry::USED_MUTABLE);
    return llvm::cast<PabloAST>(assign);
}


PabloAST * SymbolTable::lookup(Token * identifier) {
    assert (identifier->getType() == TokenType::IDENTIFIER);
    std::string name = identifier->getText();
    auto optEntry = localFind(name);
    if (!optEntry) {
        optEntry = higherFind(name);
        if (optEntry && !optEntry.value().attr[Entry::MUTABLE]) {
            // Found an entry in a higher symbol table, but it was not a mutable
            // symbol. In this case, we don't want to overwrite the symbol so
            // treat it like we didn't find one at all.
            optEntry = boost::none;
        }
    }
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
    if (!e.attr[Entry::INDEXABLE] || !e.attr[Entry::MUTABLE]) {
        mErrorManager->logError(identifier, errtxt_NonIndexableSymbol(name));
        mErrorManager->logNote(e.token, errtxt_DefinitionNote(name));
        return nullptr;
    }
    if (index->getType() != TokenType::INT_LITERAL) {
        mErrorManager->logError(index, "non-integer indexing is not supported at this time");
        return nullptr;
    }
    auto idx = (int64_t) index->getValue();
    PabloAST * const var = e.value;
    assert (llvm::isa<Var>(var));
    return mBuilder->createExtract(llvm::cast<Var>(var), idx);
}


void SymbolTable::addInputVar(Token * identifier, PabloKernelSignature::Type * type, PabloSourceKernel * kernel) {
    std::string name = identifier->getText();
    PabloAST * var = nullptr;
    Entry e;
    if (llvm::isa<PabloKernelSignature::IntType>(type)) {
        mErrorManager->logFatalError(identifier, "input scalars are not supported in pablo kernels");
    } else if (llvm::isa<PabloKernelSignature::StreamType>(type)) {
        var = kernel->getInputStreamVar(name);
        e = Entry(var, identifier, {Entry::INPUT, Entry::MUTABLE, Entry::HAS_VALUE});
    } else if (llvm::isa<PabloKernelSignature::StreamSetType>(type)) {
        var = kernel->getInputStreamVar(name);
        e = Entry(var, identifier, {Entry::INPUT, Entry::MUTABLE, Entry::HAS_VALUE, Entry::INDEXABLE});
    } else {
        llvm_unreachable("invalid type");
    }
    assert (e.value != nullptr && e.token != nullptr);
    auto rt = mEntries.insert({name, std::move(e)});
    if (!rt.second) {
        mErrorManager->logFatalError(identifier, errtxt_RedefinitionError(name));
        auto prev = mEntries[name];
        mErrorManager->logNote(prev.token, errtxt_PreviousDefinitionNote());
    }
}


void SymbolTable::addOutputVar(Token * identifier, PabloKernelSignature::Type * type, PabloSourceKernel * kernel) {
    std::string name = identifier->getText();
    PabloAST * var = nullptr;
    Entry e;
    if (llvm::isa<PabloKernelSignature::IntType>(type)) {
        var = kernel->getOutputScalarVar(identifier->getText());
        e = Entry(var, identifier, {Entry::OUTPUT, Entry::MUTABLE});
    } else if (llvm::isa<PabloKernelSignature::StreamType>(type)) {
        var = kernel->getOutputStreamVar(identifier->getText());
        e = Entry(var, identifier, {Entry::OUTPUT, Entry::MUTABLE});
    } else if (llvm::isa<PabloKernelSignature::StreamSetType>(type)) {
        var = kernel->getOutputStreamVar(identifier->getText());
        e = Entry(var, identifier, {Entry::OUTPUT, Entry::MUTABLE, Entry::INDEXABLE});
    } else {
        llvm_unreachable("invalid type");
    }
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
