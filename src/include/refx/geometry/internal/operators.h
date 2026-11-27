#ifndef _REFX_GEOMETRY_INTERNAL_OPERATORS_
#define _REFX_GEOMETRY_INTERNAL_OPERATORS_

#include <array>
#include <cmath>
#include <cstddef>

#include "../../frames/internal/traits.h"
#include "../../math/angles.h"

namespace refx {
namespace internal {

/**
 * @file operators.h
 * @brief Defines the low-level, trait-aware arithmetic engine for frame-aware types.
 * @details This file contains the internal utilities that power the arithmetic of
 * `Vector3D` and `Coordinate3D`. It is not intended for direct use by the end-user.
 *
 * The engine is split in two parts, reflecting the different mathematical nature of the
 * two entities:
 *
 * - `FramedVecOperator` implements the arithmetic of **vectors** (`Vector3D`). A vector is an
 *   element of a vector space (or, for non-Cartesian frames such as `lla` and `aer`, an
 *   *increment* of the coordinates). Its arithmetic is therefore always plain component-wise
 *   linear algebra: no wrapping or normalization is ever applied. For example, a longitude
 *   increment of 190° is a legitimate increment, and doubling a latitude increment of 50°
 *   yields 100°.
 *
 * - `CoordinateOperator` implements the affine operations involving **coordinates**
 *   (`Coordinate3D`):
 *   - `difference(a, b)`: `a - b`, the increment that brings `b` to `a`. Periodic components
 *     (e.g. longitude, azimuth) use the shortest-angle difference.
 *   - `translate(a, d)`: `a + d`, the coordinate reached by applying the increment `d` to `a`.
 *     The result is normalized into the canonical domain of the frame: bounded components
 *     (e.g. latitude, elevation) that cross a pole/zenith are reflected back, shifting the
 *     coupled periodic component (longitude, azimuth) by half a turn; periodic components are
 *     wrapped into their range.
 *
 * Both rely on the `AxisDomain` of each component, provided by `FrameTraits<Frame>`. For
 * Cartesian frames all the components are `AxisDomain::Linear` and all the operations reduce
 * to the standard component-wise arithmetic.
 */

/*==========================================================================*/
/*====================== AxisDomain compile-time helpers ===================*/
/*==========================================================================*/

/// @brief True for bounded angular domains (e.g. latitude, elevation): `[-h, +h]`.
constexpr bool is_bounded_angular(AxisDomain A) {
    return A == AxisDomain::WrappedAngular90 || A == AxisDomain::WrappedAngularPi2;
}

/// @brief True for periodic angular domains (e.g. longitude, azimuth).
constexpr bool is_periodic_angular(AxisDomain A) {
    return A == AxisDomain::WrappedAngular180 || A == AxisDomain::WrappedAngular360 ||
           A == AxisDomain::WrappedAngularPi || A == AxisDomain::WrappedAngular2Pi;
}

/// @brief True for domains expressed in radians.
constexpr bool is_radian_domain(AxisDomain A) {
    return A == AxisDomain::WrappedAngularPi2 || A == AxisDomain::WrappedAngularPi ||
           A == AxisDomain::WrappedAngular2Pi;
}

/// @brief Half a turn (180° or pi) in the unit of the given domain.
template <AxisDomain A, typename T>
constexpr T half_turn() {
    return is_radian_domain(A) ? pi_v<T> : T(180);
}

/// @brief Wraps `v` into `[lo, lo + period)`.
template <typename T>
T wrap_into(T v, T lo, T period) {
    T r = std::fmod(v - lo, period);
    if (r < T(0)) r += period;
    return r + lo;
}

/// @brief Wraps `v` into the canonical range of the periodic domain `A`:
/// `[-180, 180)`, `[0, 360)`, `[-pi, pi)` or `[0, 2pi)`.
template <AxisDomain A, typename T>
T wrap_periodic(T v) {
    constexpr T half = half_turn<A, T>();
    if constexpr (A == AxisDomain::WrappedAngular180 || A == AxisDomain::WrappedAngularPi) {
        return wrap_into(v, -half, T(2) * half);
    } else {
        return wrap_into(v, T(0), T(2) * half);
    }
}

/// @brief Shortest signed angular difference `a - b`, in `[-half turn, +half turn)`.
template <AxisDomain A, typename T>
T shortest_difference(T a, T b) {
    constexpr T half = half_turn<A, T>();
    return wrap_into(a - b, -half, T(2) * half);
}

/// @brief Index of the first component with the given property, or 3 if none.
template <typename Frame, typename Pred>
constexpr std::size_t find_axis(Pred pred) {
    for (std::size_t i = 0; i < 3; ++i) {
        if (pred(FrameTraits<Frame>::axis[i])) return i;
    }
    return 3;
}

/*==========================================================================*/
/*============================ Vector arithmetic ===========================*/
/*==========================================================================*/

/**
 * @brief Component-wise arithmetic engine for `Vector3D`.
 * @tparam FramedVecType The vector type (e.g. `Vector3D<ned, double>`). It must expose the
 * `::scalar_type` typedef and the `.x()`, `.y()`, `.z()` accessors.
 *
 * @details Vectors (displacements, velocities, ... or coordinate increments for non-Cartesian
 * frames) always obey plain linear arithmetic: no angular normalization is applied here.
 *
 * @warning This is a low-level implementation detail and should not be invoked
 * directly. Use the standard C++ operators (`+`, `-`, `*`, `/`) instead.
 */
template <typename FramedVecType>
struct FramedVecOperator {
    using T = typename FramedVecType::scalar_type;

    static FramedVecType add(const FramedVecType& a, const FramedVecType& b) {
        return FramedVecType(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
    }

    static FramedVecType sub(const FramedVecType& a, const FramedVecType& b) {
        return FramedVecType(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
    }

    static FramedVecType scalar_mul(const FramedVecType& a, T b) {
        return FramedVecType(a.x() * b, a.y() * b, a.z() * b);
    }

    static FramedVecType scalar_mul(T a, const FramedVecType& b) { return scalar_mul(b, a); }

    static FramedVecType scalar_div(const FramedVecType& a, T b) {
        return FramedVecType(a.x() / b, a.y() / b, a.z() / b);
    }
};

/*==========================================================================*/
/*========================= Coordinate arithmetic ==========================*/
/*==========================================================================*/

/**
 * @brief Trait-aware engine for the affine operations on coordinates.
 * @tparam Frame The frame of the coordinates.
 * @tparam T The scalar type.
 *
 * @details See the file documentation for the semantics. All the angular quantities are
 * expressed in the unit of the respective `AxisDomain` (degrees for `WrappedAngular90/180/360`,
 * radians for `WrappedAngularPi2/Pi/2Pi`).
 *
 * @warning This is a low-level implementation detail and should not be invoked directly.
 */
template <typename Frame, typename T>
struct CoordinateOperator {
    using array_type = std::array<T, 3>;

    /// @brief The increment `a - b` (shortest-angle for periodic components).
    static array_type difference(const array_type& a, const array_type& b) {
        return {diff_component<0>(a[0], b[0]), diff_component<1>(a[1], b[1]),
                diff_component<2>(a[2], b[2])};
    }

    /// @brief The coordinate `a + d`, normalized into the canonical domain of the frame.
    static array_type translate(const array_type& a, const array_type& d) {
        return normalize({a[0] + d[0], a[1] + d[1], a[2] + d[2]});
    }

    /**
     * @brief Brings a coordinate into the canonical domain of the frame.
     * @details A bounded component beyond its limit (e.g. a latitude of 100°) is reflected
     * back (80°) and the coupled periodic component (longitude) is shifted by half a turn,
     * which describes the same point. Periodic components are then wrapped into their range.
     */
    static array_type normalize(array_type c) {
        constexpr std::size_t ib = find_axis<Frame>(is_bounded_angular);
        constexpr std::size_t ip = find_axis<Frame>(is_periodic_angular);

        if constexpr (ib < 3) {
            constexpr AxisDomain AB = FrameTraits<Frame>::axis[ib];
            constexpr T half = half_turn<AB, T>();  // 180° (or pi)
            constexpr T limit = half / T(2);        // 90° (or pi/2)
            // A bounded angle is periodic over a full turn: bring it into [-half, half).
            T v = wrap_into(c[ib], -half, T(2) * half);
            bool crossed = false;
            if (v > limit) {
                v = half - v;
                crossed = true;
            } else if (v < -limit) {
                v = -half - v;
                crossed = true;
            }
            c[ib] = v;
            if constexpr (ip < 3) {
                if (crossed) c[ip] += half_turn<FrameTraits<Frame>::axis[ip], T>();
            }
        }
        if constexpr (ip < 3) {
            c[ip] = wrap_periodic<FrameTraits<Frame>::axis[ip]>(c[ip]);
        }
        return c;
    }

   private:
    template <std::size_t I>
    static T diff_component(T a, T b) {
        constexpr AxisDomain A = FrameTraits<Frame>::axis[I];
        if constexpr (is_periodic_angular(A)) {
            return shortest_difference<A>(a, b);
        } else {
            // Linear and bounded components: plain difference (a latitude difference is
            // always within [-180°, 180°] and never wraps).
            return a - b;
        }
    }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_GEOMETRY_INTERNAL_OPERATORS_ */
