#ifndef BOUNDARIES_H
#define BOUNDARIES_H

namespace re {
    
class RE;
class Name;

bool hasGraphemeClusterBoundary(const RE * re);

bool hasWordBoundary(const RE * re);

RE * resolveGraphemeMode(RE * re, bool inGraphemeMode);

RE * generateGraphemeClusterBoundaryRule(bool extendedGraphemeClusters = true);

}

#endif

