#ifndef _REFX_INTERNAL_ALWAYS_FALSE_
#define _REFX_INTERNAL_ALWAYS_FALSE_

/**
 * @file always_false.h
 * @brief Dependent-false helpers for `static_assert` in templates.
 * @details Before C++23 (P2593), `static_assert(false, ...)` inside a template is ill-formed
 * (no diagnostic required) even in a branch that is never instantiated, and compilers such as
 * GCC <= 12 and Clang <= 16 reject it eagerly. Making the condition depend on a template
 * parameter defers the check to instantiation time, which is valid C++17.
 *
 * Usage:
 * @code
 * static_assert(internal::always_false_v<T>, "message");      // type parameter(s)
 * static_assert(internal::always_false_value_v<D>, "message"); // non-type parameter(s)
 * @endcode
 */

namespace refx {
namespace internal {

/// Always `false`, but dependent on the given type template parameters.
template <typename...>
inline constexpr bool always_false_v = false;

/// Always `false`, but dependent on the given non-type template parameters.
template <auto...>
inline constexpr bool always_false_value_v = false;

}  // namespace internal
}  // namespace refx

#endif  // _REFX_INTERNAL_ALWAYS_FALSE_
