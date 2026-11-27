#ifndef _REFX_MATH_ANGLES_
#define _REFX_MATH_ANGLES_

#include <cmath>
#include <tuple>
#include <type_traits>

/**
 * @file angles.h
 * @brief Provides essential mathematical utilities for angular unit conversions.
 * @details This header contains a set of lightweight, inline helper functions for
 * converting between the common angular representations used in robotics and geodesy:
 * decimal degrees, radians, and the Degrees/Minutes/Seconds (DMS) format.
 *
 * These utilities are fundamental to the library, as they ensure that angles can be
 * correctly passed between different components, such as user inputs, internal
 * calculations (which often require radians), and external interfaces.
 */

namespace refx {

namespace internal {
/// @brief The constant pi. Defined here so the library does not rely on the non-standard `M_PI`
/// macro (which requires `_USE_MATH_DEFINES` on MSVC).
template <typename T>
inline constexpr T pi_v = static_cast<T>(3.141592653589793238462643383279502884L);

/// @brief Floating-point type used for angle conversions: integral inputs are promoted to
/// `double`, floating-point inputs keep their own type.
template <typename T>
using angle_float_t = std::conditional_t<std::is_integral_v<T>, double, T>;
}  // namespace internal

/**
 * @brief Specifies units for angular quantities, used in accessor methods.
 */
enum class AngleUnit {
    Deg,  ///< Angle is expressed in degrees.
    Rad   ///< Angle is expressed in radians.
};

/**
 * @brief Converts an angle from decimal degrees to radians.
 * @details This is a fundamental conversion required for nearly all standard C++
 * trigonometric functions (e.g., `std::sin`, `std::cos`), which expect their
 * inputs to be in radians.
 * @tparam T The scalar type of the angle (e.g., `float`, `double`).
 * @param deg The angle in decimal degrees.
 * @return The equivalent angle in radians. Integral inputs are promoted to `double`.
 */
template <typename T>
inline internal::angle_float_t<T> deg2rad(T deg) {
    using R = internal::angle_float_t<T>;
    return static_cast<R>(deg) * (internal::pi_v<R> / R(180));
}

/**
 * @brief Converts an angle from radians to decimal degrees.
 * @details This is useful for converting the output of trigonometric functions
 * (like `std::atan2`) or internal radian-based states into a more human-readable
 * degree format, which is standard for geodetic coordinates (latitude/longitude).
 * @tparam T The scalar type of the angle (e.g., `float`, `double`).
 * @param rad The angle in radians.
 * @return The equivalent angle in decimal degrees. Integral inputs are promoted to `double`.
 */
template <typename T>
inline internal::angle_float_t<T> rad2deg(T rad) {
    using R = internal::angle_float_t<T>;
    return static_cast<R>(rad) * (R(180) / internal::pi_v<R>);
}

/**
 * @brief Converts an angle from decimal degrees to Degrees, Minutes, Seconds (DMS).
 * @details The DMS format is a traditional sexagesimal representation for geographic
 * coordinates. This function decomposes a floating-point degree value into its
 * integer degrees, integer minutes, and floating-point seconds components.
 * @tparam T The scalar type of the input angle.
 * @param decimal_deg The angle in decimal degrees.
 * @return A `std::tuple<int, int, T>` containing the (degrees, minutes, seconds).
 * @note The sign of a negative angle is carried by the first non-zero component, so that
 * angles in (-1°, 0°) are representable: e.g. -0.5° -> (0, -30, 0).
 */
template <typename T>
inline std::tuple<int, int, T> degToDMS(T decimal_deg) {
    const bool negative = std::signbit(decimal_deg);
    const T abs_deg = std::abs(decimal_deg);
    int deg = static_cast<int>(abs_deg);
    const T minutes_total = (abs_deg - deg) * T(60);
    int min = static_cast<int>(minutes_total);
    T sec = (minutes_total - min) * T(60);
    if (negative) {
        if (deg != 0) {
            deg = -deg;
        } else if (min != 0) {
            min = -min;
        } else {
            sec = -sec;
        }
    }
    return {deg, min, sec};
}

/**
 * @brief Converts an angle from Degrees, Minutes, Seconds (DMS) to decimal degrees.
 * @details This function composes a decimal degree value from its integer degrees,
 * integer minutes, and floating-point seconds components. It correctly handles
 * negative degree values.
 * @tparam T The scalar type used for the seconds component and for intermediate calculations.
 * @param deg The integer degrees component. Can be negative.
 * @param min The integer minutes component. Can be negative only when `deg == 0`.
 * @param sec The floating-point seconds component. Can be negative only when `deg == 0` and
 * `min == 0`.
 * @return The equivalent angle in decimal degrees.
 * @note The result is negative if any of the components is negative (this is the inverse of
 * `degToDMS`, which places the sign on the first non-zero component).
 */
template <typename T>
inline T DMSToDeg(int deg, int min, T sec) {
    const bool negative = deg < 0 || min < 0 || std::signbit(sec);
    const T magnitude = static_cast<T>(std::abs(deg)) + static_cast<T>(std::abs(min)) / T(60) +
                        std::abs(sec) / T(3600);
    return negative ? -magnitude : magnitude;
}

}  // namespace refx

#endif /* _REFX_MATH_ANGLES_ */
