#ifndef ENUM_FLAGS_HPP
#define ENUM_FLAGS_HPP

#include <type_traits>
namespace {
template<typename EnumType>
struct __EnumFlagSet {
    static const bool enable = false;
};
}

#define ENABLE_ENUM_FLAGS(EnumType) \
namespace { \
template<> \
struct __EnumFlagSet<EnumType> { \
    static const bool enable = true; \
}; \
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType>::type
operator|(EnumType lhs, EnumType rhs) {
    using IntType = typename std::underlying_type<EnumType>::type;
    return static_cast<EnumType>(static_cast<IntType>(lhs) | static_cast<IntType>(rhs));
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType>::type
operator&(EnumType lhs, EnumType rhs) {
    using IntType = typename std::underlying_type<EnumType>::type;
    return static_cast<EnumType>(static_cast<IntType>(lhs) & static_cast<IntType>(rhs));
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType>::type
operator^(EnumType lhs, EnumType rhs) {
    using IntType = typename std::underlying_type<EnumType>::type;
    return static_cast<EnumType>(static_cast<IntType>(lhs) ^ static_cast<IntType>(rhs));
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType>::type
operator~(EnumType lhs){
    using IntType = typename std::underlying_type<EnumType>::type;
    return static_cast<EnumType>(~static_cast<IntType>(lhs));
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType&>::type
operator|=(EnumType & lhs, EnumType rhs) {
    using IntType = typename std::underlying_type<EnumType>::type;
    return lhs = static_cast<EnumType>(static_cast<IntType>(lhs) | static_cast<IntType>(rhs));
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType&>::type
operator&=(EnumType & lhs, EnumType rhs) {
    using IntType = typename std::underlying_type<EnumType>::type;
    return lhs = static_cast<EnumType>(static_cast<IntType>(lhs) & static_cast<IntType>(rhs));
}

template<typename EnumType>
constexpr typename std::enable_if<__EnumFlagSet<EnumType>::enable, EnumType&>::type
operator^=(EnumType & lhs, EnumType rhs) {
    using IntType = typename std::underlying_type<EnumType>::type;
    return lhs = static_cast<EnumType>(static_cast<IntType>(lhs) ^ static_cast<IntType>(rhs));
}

#endif
