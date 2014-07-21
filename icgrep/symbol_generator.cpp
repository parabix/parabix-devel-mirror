/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "symbol_generator.h"

SymbolGenerator::SymbolGenerator(){
	pfxmap = new std::map<std::string, int>();
	//std::cout << "pfxmap initialized" << std::endl;
}

std::string SymbolGenerator::gensym(std::string prefix)
{

    std::pair<std::map<std::string, int>::iterator, bool> ret = pfxmap->insert(make_pair(prefix, 0));
    std::string sym;
    if (ret.second)
    {
        //The insertion succeeded
        sym = prefix + INT2STRING(0);
        //std::cout << sym << " created" << std::endl;
        return sym;
    }
    else
    {
        //The insertion failed so we know that the prefix has already been added.
        std::map<std::string, int>::iterator iter = pfxmap->find(prefix);
        iter->second++;

        sym = prefix + INT2STRING(iter->second);
        //std::cout << sym << " created" << std::endl;
        return sym;
	
    }
}
