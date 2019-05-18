/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <boost/make_unique.hpp>
#include <pablo/parser/error.h>
#include <pablo/parser/lexer.h>
#include <pablo/parser/rd_parser.h>
#include <pablo/parser/simple_lexer.h>
#include <pablo/parser/pablo_parser.h>

using namespace pablo::parse;

int main(int argc, char ** argv) {
    if (argc != 3) {
        std::cerr << "a command and input file is required\n";
        return 1;
    }

    std::string filename{argv[2]};
    auto errorDelegate = std::make_shared<ErrorManager>();
    auto source = std::make_shared<SourceFile>(filename);

    // test lexer
    if (std::strcmp(argv[1], "tokenize") == 0) {
        SimpleLexer lexer(errorDelegate);
        auto tokens = lexer.tokenize(source);
        if (!tokens) {
            errorDelegate->dumpErrors();
            return 1;
        }

        for (auto const & token : tokens.value()) {
            std::cout << *token << "\n";
        }
    }

    // test parser
    else if (std::strcmp(argv[1], "parse") == 0) {
        auto lexer = boost::make_unique<SimpleLexer>(errorDelegate);
        RecursiveParser parser(std::move(lexer), errorDelegate);
        parser.parse(source);
    }

    else {
        std::cerr << "unknown command: " << argv[1] << "\n";
        return 1;
    }

    return 0;
}
