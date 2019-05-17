/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <fstream>
#include <pablo/parser/error.h>
#include <pablo/parser/simple_lexer.h>
#include <pablo/parser/pablo_parser.h>

using namespace pablo::parse;

int main(int argc, char ** argv) {
    if (argc != 2) {
        std::cerr << "input file is required\n";
        return 1;
    }
    std::string filename{argv[1]};
    SimpleLexer lexer{ErrorContext{}};
    SourceFile file(filename);
    auto tokens = lexer.tokenize(file);
    if (tokens == nullptr) {
        lexer.getErrorManager().dumpErrors();
        return 1;
    }

    for (auto const & token : *tokens) {
        std::cout << *token << "\n";
    }
}
