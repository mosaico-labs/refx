#ifndef _REFX_FRAMES_INTERNAL_TRAITS_
#define _REFX_FRAMES_INTERNAL_TRAITS_

#include <type_traits>

#include "../axis.h"
#include "../frames.h"
/**
 * @file traits.h
 * @brief Defines the default compile-time traits for the frames.
 * @details This file establishes the foundational `refx::FrameTraits` class template.
 * Its primary role is to define the **default behavior** for all frame-aware entities
 * in the library.
 *
 * The core design principle is to assume that any given coordinate frame is **Cartesian**
 * (composed of three orthogonal, linear axes) unless explicitly specified otherwise.
 * This makes the library easily extensible, as new Cartesian frames can be added
 * without needing to create a corresponding trait specialization.
 *
 * Specializations for non-Cartesian frames (like `lla` or `aer`) are provided to override this
 * default behavior to enable mathematically correct operations on angular or non-linear components.
 */

namespace refx {
namespace internal {

/**
 * @brief Default compile-time traits for a generic 3D vector.
 * @tparam Frame The coordinate frame tag type (e.g., `ned`, `frd`, `wa`).
 *
 * @details This class template serves as a metadata provider for `Vector3D` and other
 * geometric types. It allows generic algorithms to query the physical properties of
 * a vector's components at compile time.
 *
 * The default implementation provided here assumes the most common case in robotics:
 * a **right-handed Cartesian coordinate system**. All three axes are treated as
 * `AxisDomain::Linear`, meaning they represent distances that can be added, subtracted,
 * and scaled using standard arithmetic. This default applies to frames like `ned`, `enu`,
 * `frd`, `ecef`, and any other frame without an explicit specialization.
 *
 * @note This base template should not be modified. To support a non-Cartesian frame,
 * provide a template specialization in a different, more specific header file.
 */
template <typename Frame>
struct FrameTraits {
    /**
     * @brief An array defining the physical type of each of the frame's three axes. 📏
     *
     * @details By default, all axes are `AxisDomain::Linear`. This informs the library's
     * operators and functions that standard algebra on frame-aware vectors and coordinates is safe
     * and correct for this frame.
     *
     * The mapping is consistent across all vector types:
     * - `axis[0]`: Corresponds to the first component (e.g., X, North, Forward).
     * - `axis[1]`: Corresponds to the second component (e.g., Y, East, Right).
     * - `axis[2]`: Corresponds to the third component (e.g., Z, Down, Up).
     */
    static constexpr AxisDomain axis[3] = {AxisDomain::Linear, AxisDomain::Linear,
                                           AxisDomain::Linear};
};

/*==========================================================================*/
/*=========== VECTOR TRAIT SPECIALIZATIONS FOR NON-LINEAR FRAMES ===========*/
/*==========================================================================*/

/**
 * @details The following section provides specializations of the `refx::FrameTraits`
 * template for coordinate systems that include non-linear or angular components,
 * such as geodetic (LLA) and spherical (AER) frames.
 *
 * The primary purpose of these traits is to inform the library's generic arithmetic
 * engine (`FramedVecOperator`) how to correctly handle each component of a vector
 * or coordinate. Standard vector subtraction, for example, is mathematically
 * incorrect for angles that wrap around (like longitude).
 *
 * By defining each axis with a specific `AxisDomain` (e.g., `WrappedAngular180`), we enable
 * the underlying operators to apply specialized, mathematically correct logic at
 * **compile-time**. This prevents subtle but critical bugs in navigation, sensor
 * fusion, and robotics algorithms.
 *
 * @note Standard Cartesian frames (e.g., `ned`, `frd`) do not require specialization
 * here because the default `FrameTraits` template correctly assumes all their axes
 * are `AxisDomain::Linear`.
 */

/**
 * @brief Traits for the **Latitude, Longitude, Altitude (LLA)** geodetic frame.
 *
 * @details Defines the physical nature of each axis in the LLA coordinate system.
 *
 * The axes are characterized as follows:
 * - **[0] Latitude:** An angle clamped between -90° (South Pole) and +90° (North Pole).
 * Marked as `AxisDomain::WrappedAngular90`.
 * - **[1] Longitude:** An angle that wraps around at ±180°. This trait ensures that
 * differences are calculated along the shortest path (e.g., across the antimeridian).
 * Marked as `AxisDomain::WrappedAngular180`.
 * - **[2] Altitude:** A linear distance in meters above a reference ellipsoid. Standard
 * arithmetic applies. Marked as `AxisDomain::Linear`.
 */
template <>
struct FrameTraits<lla> {
    /// @brief Specifies the type of each axis for LLA coordinates.
    static constexpr AxisDomain axis[3] = {
        AxisDomain::WrappedAngular90,   ///< Latitude is clamped to [-90°, 90°].
        AxisDomain::WrappedAngular180,  ///< Longitude wraps around at [-180°, 180°].
        AxisDomain::Linear              ///< Altitude is a linear distance in meters.
    };
};

/**
 * @brief Traits for the **Latitude, Longitude, Down (LLD)** geodetic frame.
 *
 * @details Similar to LLA, this frame is used for georeferenced positioning but uses a "Down"
 * component instead of "Altitude".
 *
 * The axes are characterized as follows:
 * - **[0] Latitude:** An angle clamped to `[-90°, +90°]`. Marked as `AxisDomain::WrappedAngular90`.
 * - **[1] Longitude:** An angle wrapping at `±180°`. Marked as `AxisDomain::WrappedAngular180`.
 * - **[2] Down:** A linear distance in meters, representing depth below a reference ellipsoid.
 * Marked as `AxisDomain::Linear`.
 */
template <>
struct FrameTraits<lld> {
    /// @brief Specifies the type of each axis for LLD coordinates.
    static constexpr AxisDomain axis[3] = {
        AxisDomain::WrappedAngular90,   ///< Latitude is clamped to [-90°, 90°].
        AxisDomain::WrappedAngular180,  ///< Longitude wraps around at [-180°, 180°].
        AxisDomain::Linear              ///< Down/Depth is a linear distance in meters.
    };
};

/**
 * @brief Traits for the **Azimuth, Elevation, Range (AER)** spherical frame.
 *
 * @details Describes the axes of a spherical coordinate system.
 *
 * The axes are characterized as follows:
 * - **[0] Azimuth:** A horizontal angle, typically measured clockwise from a reference
 * direction (like North), that wraps around at 360°. Marked as `AxisDomain::WrappedAngular360`.
 * - **[1] Elevation:** A vertical angle measured from the horizontal plane, clamped
 * between -90° (nadir) and +90° (zenith). Marked as `AxisDomain::WrappedAngular90`.
 * - **[2] Range:** The straight-line Euclidean distance from the origin to the point.
 * Marked as `AxisDomain::Linear`.
 */
template <>
struct FrameTraits<aer> {
    /// @brief Specifies the type of each axis for AER coordinates.
    static constexpr AxisDomain axis[3] = {
        AxisDomain::WrappedAngular360,  ///< Azimuth wraps around at [0°, 360°).
        AxisDomain::WrappedAngular90,   ///< Elevation is clamped to [-90°, 90°].
        AxisDomain::Linear              ///< Range is a linear distance in meters.
    };
};

// Primary template: assume no typedefs
template <typename, typename = void>
struct has_axis_and_tag : std::false_type {};

// Specialization: valid if T::axis and T::tag exist
template <typename T>
struct has_axis_and_tag<T, std::void_t<typename T::axis, decltype(T::tag)>> : std::true_type {};

template <typename T>
inline constexpr bool has_axis_and_tag_v = has_axis_and_tag<T>::value;

/**
 * @brief A helper struct to extract the template parameters from a DirectionalAxis type.
 * @details This trait allows us to access the X, Y, and Z directions of an axis
 * type in our constexpr functions.
 */
template <typename Axis>
struct AxisTraits;  // Primary template is undefined

template <AxisDirection x, AxisDirection y, AxisDirection z>
struct AxisTraits<DirectionalAxis<x, y, z>> {
    static constexpr AxisDirection X = x;
    static constexpr AxisDirection Y = y;
    static constexpr AxisDirection Z = z;
};

// Primary templates default to false.
template <typename T>
struct is_semantic_axis : std::false_type {};
template <typename T>
struct is_directional_axis : std::false_type {};

// Partial specializations for any valid axis type.
template <AxisSemantic x, AxisSemantic y, AxisSemantic z>
struct is_semantic_axis<SemanticAxis<x, y, z>> : std::true_type {};

template <AxisDirection x, AxisDirection y, AxisDirection z>
struct is_directional_axis<DirectionalAxis<x, y, z>> : std::true_type {};
}  // namespace internal

/**
 * @brief A C++17 variable template that is true if T is a SemanticAxis.
 * @details This trait is a key tool for static polymorphism, allowing functions
 * and classes to adapt their behavior at compile time based on whether a frame
 * is non-Cartesian.
 */
template <typename FrameAxis>
inline constexpr bool is_semantic_axis_v = internal::is_semantic_axis<FrameAxis>::value;

/**
 * @brief A C++17 variable template that is true if T is a DirectionalAxis.
 * @details This trait allows algorithms to verify at compile time that a frame
 * is Cartesian before attempting geometric operations like cross products.
 */
template <typename FrameAxis>
inline constexpr bool is_directional_axis_v = internal::is_directional_axis<FrameAxis>::value;

}  // namespace refx

#endif /* _REFX_FRAMES_INTERNAL_TRAITS_ */
