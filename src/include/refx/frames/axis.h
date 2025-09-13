#ifndef _REFX_FRAMES_AXIS_
#define _REFX_FRAMES_AXIS_

#include <type_traits>  // For std::true_type, std::false_type

/**
 * @file axis.h
 * @brief Defines the architectural building blocks for coordinate frame axes.
 * @details This foundational header establishes the type system used to describe the
 * structure and properties of all coordinate frames in the library.
 *
 * The core architectural philosophy is the **separation of concerns** between a
 * frame's semantic name (e.g., `ned`) and its underlying axis structure. This file
 * introduces two parallel type systems for this purpose:
 *
 * 1.  **DirectionalAxis**: For right-handed Cartesian frames, this system uses
 * geometric directions (`Forw`, `Right`, `Up`, etc.) to define the orientation
 * of the X, Y, and Z axes.
 *
 * 2.  **SemanticAxis**: For non-Cartesian frames (e.g., geodetic or spherical), this
 * system uses semantic tags (`Latitude`, `Altitude`, `Range`, etc.) to
 * describe the physical meaning of each component.
 *
 * This dual system enables powerful compile-time validation and allows generic
 * algorithms to adapt their behavior based on whether a frame is Cartesian or not.
 * This file also defines `AxisDomain`, a critical enum that informs the library's
 * compile-time arithmetic engine about the mathematical properties of each component.
 */

namespace refx {

//==============================================================================
// Component Mathematical Properties
//==============================================================================

/**
 * @brief Defines the mathematical domain and properties of a single component.
 * @details This enumeration is a key part of the library's metaprogramming
 * infrastructure. It is used by the `FrameTraits` struct to tag each component
 * of a vector or coordinate, allowing generic functions like the arithmetic
 * operators to apply mathematically correct logic at compile-time.
 *
 * For example, when subtracting two coordinates, this enum allows the operator to
 * perform standard linear subtraction for a `Linear` component but apply
 * specialized shortest-angle logic for a `WrappedAngular180` component (like
 * longitude), thus preventing critical errors in navigation calculations.
 */
enum class AxisDomain {
    /**
     * @brief A standard linear, Euclidean component where standard arithmetic applies.
     * @details This is the default for all components in Cartesian frames.
     * Examples: North component in NED, Altitude in LLA.
     */
    Linear,

    /**
     * @brief An angular component that is clamped to a 180-degree total range
     * (e.g., `[-90°, +90°]`).
     * @details Arithmetic operations may involve clamping or normalization.
     * Example: Geodetic latitude, Elevation.
     */
    WrappedAngular90,

    /**
     * @brief An angular component that wraps around at `±180°`.
     * @details Arithmetic on this component, especially subtraction, must use
     * shortest-angle logic to handle the discontinuity.
     * Example: Geodetic longitude.
     */
    WrappedAngular180,

    /**
     * @brief An angular component that wraps around at `360°`, typically in the
     * range `[0°, 360°)`.
     * @details Arithmetic involves modulo operations to handle the wrap-around.
     * Example: Azimuth.
     */
    WrappedAngular360
};

//==============================================================================
// Directional (Cartesian) Axis System
//==============================================================================

/**
 * @brief Defines the six cardinal directions for constructing Cartesian axes.
 */
enum class AxisDirection { Up, Down, Forw, Back, Left, Right };

/**
 * @brief Helper function to find the geometric opposite of an AxisDirection.
 * @param dir The input direction.
 * @return The opposing direction (e.g., Up -> Down).
 */
template <AxisDirection D>
constexpr AxisDirection opposite() {
    if constexpr (D == AxisDirection::Up) {
        return AxisDirection::Down;
    } else if constexpr (D == AxisDirection::Down) {
        return AxisDirection::Up;
    } else if constexpr (D == AxisDirection::Forw) {
        return AxisDirection::Back;
    } else if constexpr (D == AxisDirection::Back) {
        return AxisDirection::Forw;
    } else if constexpr (D == AxisDirection::Left) {
        return AxisDirection::Right;
    } else if constexpr (D == AxisDirection::Right) {
        return AxisDirection::Left;
    } else {
        // Should be unreachable for valid inputs
        static_assert(false, "Invalid AxisDirection input parameter.");
    }
}

/**
 * @brief A template struct that defines a right-handed Cartesian axis system
 * using three orthogonal geometric directions.
 * @details This is a compile-time "tag" struct used to define the orientation
 * of frames like NED, FRD, ENU, etc.
 */
template <AxisDirection x, AxisDirection y, AxisDirection z>
struct DirectionalAxis {};

// --- Standard Right-Handed Cartesian Axis Configurations ---

// X: Forward or Backward
using axis_frd = DirectionalAxis<AxisDirection::Forw, AxisDirection::Right, AxisDirection::Down>;
using axis_flu = DirectionalAxis<AxisDirection::Forw, AxisDirection::Left, AxisDirection::Up>;
using axis_fur = DirectionalAxis<AxisDirection::Forw, AxisDirection::Up, AxisDirection::Right>;
using axis_fdl = DirectionalAxis<AxisDirection::Forw, AxisDirection::Down, AxisDirection::Left>;
using axis_bru = DirectionalAxis<AxisDirection::Back, AxisDirection::Right, AxisDirection::Up>;
using axis_bld = DirectionalAxis<AxisDirection::Back, AxisDirection::Left, AxisDirection::Down>;
using axis_bul = DirectionalAxis<AxisDirection::Back, AxisDirection::Up, AxisDirection::Left>;
using axis_bdr = DirectionalAxis<AxisDirection::Back, AxisDirection::Down, AxisDirection::Right>;

// X: Right or Left
using axis_rfu = DirectionalAxis<AxisDirection::Right, AxisDirection::Forw, AxisDirection::Up>;
using axis_rbd = DirectionalAxis<AxisDirection::Right, AxisDirection::Back, AxisDirection::Down>;
using axis_rub = DirectionalAxis<AxisDirection::Right, AxisDirection::Up, AxisDirection::Back>;
using axis_rdf = DirectionalAxis<AxisDirection::Right, AxisDirection::Down, AxisDirection::Forw>;
using axis_lfd = DirectionalAxis<AxisDirection::Left, AxisDirection::Forw, AxisDirection::Down>;
using axis_lbu = DirectionalAxis<AxisDirection::Left, AxisDirection::Back, AxisDirection::Up>;
using axis_luf = DirectionalAxis<AxisDirection::Left, AxisDirection::Up, AxisDirection::Forw>;
using axis_ldb = DirectionalAxis<AxisDirection::Left, AxisDirection::Down, AxisDirection::Back>;

// X: Up or Down
using axis_urf = DirectionalAxis<AxisDirection::Up, AxisDirection::Right, AxisDirection::Forw>;
using axis_ulb = DirectionalAxis<AxisDirection::Up, AxisDirection::Left, AxisDirection::Back>;
using axis_ufl = DirectionalAxis<AxisDirection::Up, AxisDirection::Forw, AxisDirection::Left>;
using axis_ubr = DirectionalAxis<AxisDirection::Up, AxisDirection::Back, AxisDirection::Right>;
using axis_drb = DirectionalAxis<AxisDirection::Down, AxisDirection::Right, AxisDirection::Back>;
using axis_dlf = DirectionalAxis<AxisDirection::Down, AxisDirection::Left, AxisDirection::Forw>;
using axis_dfr = DirectionalAxis<AxisDirection::Down, AxisDirection::Forw, AxisDirection::Right>;
using axis_dbl = DirectionalAxis<AxisDirection::Down, AxisDirection::Back, AxisDirection::Left>;

//==============================================================================
// Semantic (Non-Cartesian) Axis System
//==============================================================================

/**
 * @brief Defines the physical meaning of a component in a non-Cartesian frame.
 */
enum class AxisSemantic { Latitude, Longitude, Azimuth, Elevation, Altitude, Down, Range, Length };

/**
 * @brief A template struct that defines a non-Cartesian axis system using
 * three semantic component tags.
 * @details This is a compile-time "tag" struct used to define the structure
 * of frames like LLA (geodetic) or AER (spherical).
 */
template <AxisSemantic x, AxisSemantic y, AxisSemantic z>
struct SemanticAxis {};

// --- Common Non-Cartesian Axis Configurations ---

using axis_lla =
    SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Altitude>;
using axis_lld = SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Down>;
using axis_aer = SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>;

}  // namespace refx

#endif /* _REFX_FRAMES_AXIS_ */
