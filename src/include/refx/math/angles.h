#ifndef _REFX_MATH_ANGLES_
#define _REFX_MATH_ANGLES_

#include <cmath>
#include <tuple>

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
 * @return The equivalent angle in radians.
 * @note The mathematical constant `M_PI` is used for the conversion factor.
 */
template <typename T>
inline T deg2rad(T deg) {
    return deg * M_PI / 180.0;
}

/**
 * @brief Converts an angle from radians to decimal degrees.
 * @details This is useful for converting the output of trigonometric functions
 * (like `std::atan2`) or internal radian-based states into a more human-readable
 * degree format, which is standard for geodetic coordinates (latitude/longitude).
 * @tparam T The scalar type of the angle (e.g., `float`, `double`).
 * @param rad The angle in radians.
 * @return The equivalent angle in decimal degrees.
 */
template <typename T>
inline T rad2deg(T rad) {
    return rad * 180.0 / M_PI;
}

/**
 * @brief Converts an angle from decimal degrees to Degrees, Minutes, Seconds (DMS).
 * @details The DMS format is a traditional sexagesimal representation for geographic
 * coordinates. This function decomposes a floating-point degree value into its
 * integer degrees, integer minutes, and floating-point seconds components.
 * @tparam T The scalar type of the input angle.
 * @param decimal_deg The angle in decimal degrees.
 * @return A `std::tuple<int, int, T>` containing the (degrees, minutes, seconds).
 */
template <typename T>
inline std::tuple<int, int, T> degToDMS(T decimal_deg) {
    const int deg = static_cast<int>(decimal_deg);
    const T minutes_total = std::abs(decimal_deg - deg) * 60.0;
    const int min = static_cast<int>(minutes_total);
    const T sec = (minutes_total - min) * 60.0;
    return {deg, min, sec};
}

/**
 * @brief Converts an angle from Degrees, Minutes, Seconds (DMS) to decimal degrees.
 * @details This function composes a decimal degree value from its integer degrees,
 * integer minutes, and floating-point seconds components. It correctly handles
 * negative degree values.
 * @tparam T The scalar type used for the seconds component and for intermediate calculations.
 * @param deg The integer degrees component. Can be negative.
 * @param min The integer minutes component (should be positive).
 * @param sec The floating-point seconds component (should be positive).
 * @return The equivalent angle in decimal degrees.
 * @note The sign of the result is determined solely by the `deg` parameter.
 */
template <typename T>
inline T DMSToDeg(int deg, int min, T sec) {
    const T sign = (deg < 0 || std::signbit(static_cast<T>(deg))) ? -1.0 : 1.0;
    return sign * (std::abs(static_cast<T>(deg)) + static_cast<T>(min) / 60.0 + sec / 3600.0);
}

}  // namespace refx

#endif /* _REFX_MATH_ANGLES_ */
