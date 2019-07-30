/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <bitset>
#include <initializer_list>
#include <memory>
#include <unordered_map>
#include <boost/optional.hpp>
#include <pablo/parse/error.h>
#include <pablo/parse/kernel_signature.h>

namespace pablo {

class PabloAST;
class PabloBuilder;
class PabloSourceKernel;

namespace parse {

class PabloType;
class Token;

class SymbolTable {
public:
    SymbolTable() = delete;
    SymbolTable(std::shared_ptr<ErrorManager> errorDelegate, PabloBuilder * pb);
    SymbolTable(std::shared_ptr<ErrorManager> errorDelegate, PabloBuilder * pb, SymbolTable * parent);

    ~SymbolTable();

    /**
     * Assigns a value to a symbol inferred from an identifier token. 
     * 
     * If no such symbol can be deduced from the token, a new, symbol is
     * created and then assigned to.
     * 
     * @param token The identifier token to assign to (the assignee).
     * @param value The value to assign to the assignee.
     * @return The resulting AST node iff successful assignment, otherwise
     *         `nullptr` is returned.
     */
    PabloAST * assign(Token * token, PabloAST * value);

    /**
     * Assigns a value to an indexed offset of an assignee (var).
     * 
     * For example:
     *  lex[7] = ...
     *  lex.LF = ...
     * 
     * The assignee must be both mutable and indexable for a successful
     * assignment to occur.
     * 
     * - Logs an error if no symbol exists for var.
     * - Logs an error if the symbol for var is not indexable or not mutable.
     * 
     * @param var   The identifier token for the assignee.
     * @param index The token denoting the index.
     * @param value The value to assign to the index.
     * @return AST node for the assignment if successful, `nullptr` if not.
     */
    PabloAST * indexedAssign(Token * var, Token * index, PabloAST * value);

    /**
     * Looks up and returns the AST node value for the symbol corresponding to
     * a given identifier token.
     * 
     * - Logs an error if no such symbol is found.
     * 
     * @param identifier The identifier token for the symbol to lookup.
     * @return AST node for the symbol or `nullptr` if an error occurred.
     */
    PabloAST * lookup(Token * identifier);

    /**
     * Looks up the symbol for a given identifier and returns the result of a
     * pablo extract operation on the AST value on said symbol.
     * 
     * - Logs an error if no symbol exists for the given identifier.
     * - Logs an error if the symbol for identifier is not indexable or mutable.
     * 
     * @param identifier    The identifier token for the symbol to lookup.
     * @param index         The token denoting the index.
     * @return AST node for a extract operator or `nullptr` if an error occurred.
     */
    PabloAST * indexedLookup(Token * identifier, Token * index);

    /**
     * Registers a kernel input parameter in this symbol table.
     * 
     * @param identifier    The identifier token for the parameter.
     * @param type          Type of the parameter.
     * @param kernel        The kernel which the parameter belongs to.
     */
    void addInputVar(Token * identifier, PabloType * type, PabloSourceKernel * kernel);

    /**
     * Registers a kernel output parameter in this symbol table.
     * 
     * @param identifier    The identifier token for the parameter.
     * @param type          Type of the parameter.
     * @param kernel        The kernel which the parameter belongs to.
     */
    void addOutputVar(Token * identifier, PabloType * type, PabloSourceKernel * kernel);

    inline SymbolTable * getParent() const noexcept { return mParent; }

private:
    struct Entry {
        enum Attr : size_t {
            INPUT,          // is the symbol a kernel input
            OUTPUT,         // is the symbol a kernel output
            INDEXABLE,      // is the symbol indexable, either by '[]' or by '.'
            DOT_INDEXABLE,  // is the symbol indexable via '.' notation
        };

        Entry();
        Entry(Entry const & other);
        Entry(Entry && other);
        Entry(PabloAST * value, Token * token, std::initializer_list<Attr> attr = {});

        Entry & operator = (Entry const & other);
        Entry & operator = (Entry && other);

        PabloAST *      value;
        Token *         token;
        PabloType *     type;
        std::bitset<8>  attr;
        std::unordered_map<size_t, bool> setInitStatus;
    };

    // searches only this symbol table
    boost::optional<Entry> localFind(std::string const & name);

    // searches only higher symbol tables
    boost::optional<Entry> higherFind(std::string const & name);

    // searches local and higher symbol tables
    boost::optional<Entry> find(std::string const & name);

    // returns `true` if this is the symbol table for the tope level scope
    bool isTopLevelScope() const noexcept;

    std::shared_ptr<ErrorManager>           mErrorManager;
    PabloBuilder *                          mBuilder;
    std::unordered_map<std::string, Entry>  mEntries;
    SymbolTable *                           mParent;
};

} // namespace pablo::parse
} // namespace pablo
