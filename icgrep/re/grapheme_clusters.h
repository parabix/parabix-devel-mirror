#ifndef GRAPHEME_CLUSTERS_H
#define GRAPHEME_CLUSTERS_H

namespace re {
    
class RE;

bool hasGraphemeClusterBoundary(const RE * re);
    
RE * resolveGraphemeMode(RE * re, bool inGraphemeMode);

}

#endif

