#ifndef GRAPHEME_CLUSTERS_H
#define GRAPHEME_CLUSTERS_H

namespace re {
    
class RE;
    class Name;
    
bool hasGraphemeClusterBoundary(const RE * re);
    
RE * resolveGraphemeMode(RE * re, bool inGraphemeMode);

void generateGraphemeClusterBoundaryRule(Name * const &property);

}

#endif

