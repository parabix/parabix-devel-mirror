#include "propGen.h"
#include "propType.h"

#include <UCD/PropertyAliases.h>
#include <UCD/PropertyValueAliases.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <cstdlib>


using namespace std;

string getRandomPropValue(vector<string> values){
	
	if(!values.empty()){
		int random = rand() % values.size();
		return values[random];
	}
	return "";
}

string getRandomNameAlias(string prop){
	int propCode;
	std::vector<string> names;
	for (const auto &p : UCD::alias_map){
		if (p.first == prop)
			propCode = p.second;
	}
	for (const auto &p : UCD::alias_map){
		if (p.second == propCode)
			names.push_back(p.first);
	}
	if (!names.empty()){
		int random = rand() % names.size();
		return names[random];
	}
	else return prop;
}

string getBinaryProp(){

	int random = rand() % binaryProperties.size();
	return binaryProperties[random];
}

string getEnumProp(){

	int random = rand() % enumeratedProperties.size();
	int i = 0;
	for (const auto &prop : enumeratedProperties){
		if (i == random){
			string propName = prop.first;
			propName = getRandomNameAlias(propName);

			string valueParam = getRandomPropValue(prop.second);
			
			return propName + "=" + valueParam;
		}
		i++;
	}
	return "";

}

string getNumericProp(){

	int random = rand() % numericProperties.size();
	string prop =  numericProperties[random];
	prop = getRandomNameAlias(prop);
	int value = rand() % 1000;
	return prop + "=" + to_string(value);
}

string getStringProp(string regex){
	int random = rand() % stringProperties.size();
	string prop =  stringProperties[random];
	prop = getRandomNameAlias(prop);
	return prop + "=/[" + regex + "]/";
}

const string PropGen::getPropertyValue(string type, string regex){
	if (type == "binary"){
		return getBinaryProp();
	}
	else if (type == "enum"){
		return getEnumProp();
	}
	else if (type == "catalog"){
		return getEnumProp();
	}
	else if (type == "numeric"){
		return getNumericProp();
	}
	else if (type == "string"){
		return getStringProp(regex);
	}
	else {
		throw runtime_error("Unrecognized property type.");
	}
}