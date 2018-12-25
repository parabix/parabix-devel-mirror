#ifndef RE_NULLABLE_H
#define RE_NULLABLE_H

namespace re { class RE; }
namespace re { class Vector; }

namespace re {
    bool isNullable(const RE * re);
    bool isZeroWidth(const RE * re);
    RE * removeNullablePrefix(RE * re);
    RE * removeNullableSuffix(RE * re);
}

#endif // RE_NULLABLE_H
