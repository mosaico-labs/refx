#ifndef _REFX_GEOMETRY_INTERNAL_OPERATORS_
#define _REFX_GEOMETRY_INTERNAL_OPERATORS_

#include "../../frames/internal/traits.h"
// #include "math/operations.h"

namespace refx {
namespace internal {

/**
 * @file operators.h
 * @brief Defines the low-level, trait-aware arithmetic engine for frame-aware types.
 * @details This file contains the core implementation for the library's
 * specialized arithmetic and the internal utility (`FramedVecOperator`) that
 * powers all arithmetic for `Vector3D` and `Coordinate3D`.
 * It is not intended for direct use by the end-user. Instead, the global `operator+`,
 * `operator-`, etc., delegate their implementation to this struct.
 *
 * The primary purpose of this engine is to perform **compile-time dispatch** based on
 * the `FrameTraits` of a given frame. This allows the library to apply standard
 * linear arithmetic for Cartesian axes while switching to specialized
 * logic (e.g., shortest-angle subtraction) for angular axes, ensuring mathematical
 * correctness across all coordinate systems.
 */

/**
 * @brief Enumerates the basic arithmetic operations for compile-time dispatch.
 */
enum class OpType {
    Add,  ///< Represents addition (+).
    Sub,  ///< Represents subtraction (-).
    Mul,  ///< Represents multiplication (*).
    Div   ///< Represents division (/).
};

/**
 * @brief A compile-time engine for trait-aware vector arithmetic.
 * @tparam FramedVecType The frame-aware type on which to operate (e.g., `Vector3D<ned,
 * double>`). This type is expected to expose `::frame` and `::scalar_type` typedefs, as well as
 * `.x()`, `.y()`, and `.z()` accessors.
 *
 * @details This struct is the central implementation hub for all basic arithmetic
 * operations. Its static methods are called by the user-facing `operator+`, `operator-`,
 * `operator*`, and `operator/`.
 *
 * For each component of the vector, it looks up the corresponding `AxisDomain` from
 * `FrameTraits<Frame>::axis`. It then uses this type to select the correct
 * implementation of the operation at compile time (via the `apply` function).
 * This powerful mechanism is what allows the library to correctly subtract two
 * `Coordinate3D<lla>` objects by applying shortest-angle logic to the longitude
 * component while using standard subtraction for latitude and altitude.
 *
 * @warning This is a low-level implementation detail and should not be invoked
 * directly. Use the standard C++ operators (`+`, `-`, `*`, `/`) instead.
 */
template <typename FramedVecType>
struct FramedVecOperator {
    // Helper alias to extract the Frame tag from the vector type
    using Frame = typename FramedVecType::frame;
    // Helper alias to extract the scalar type (e.g., double)
    using T = typename FramedVecType::scalar_type;

    /**
     * @brief The function that performs an operation and applies trait-aware normalization.
     * @tparam A The `AxisDomain` of the operands, provided at compile-time.
     * @tparam T The scalar type (e.g., `double`, `float`).
     * @param lhs The left-hand side operand.
     * @param rhs The right-hand side operand.
     * @param op The run-time operation to perform (`Add`, `Sub`, `Mul`, `Div`).
     * @return The result of the operation, normalized according to the axis type `A`.
     *
     * @details This is the core function of the library's trait-aware arithmetic engine.
     * It works in two stages:
     * 1.  It performs the specified basic arithmetic operation (`+`, `-`, `*`, `/`).
     * 2.  It then uses the compile-time `AxisDomain` template parameter `A` to apply the
     * correct normalization logic for the result. This is a no-op for `AxisDomain::Linear`
     * but performs the necessary wrapping/clamping for angular types.
     * @warning This is a low-level utility designed to be called by `FramedVecOperator`
     * and should **not** be used directly. The logic for angular subtraction is
     * particularly subtle and handled at a higher level. All angular calculations
     * and normalizations within this function are performed assuming the inputs are
     * in **degrees**.
     * @note The normalization logic is applied to all four operations. While correct for
     * addition and subtraction, applying this type of angular normalization after
     * multiplication or division of angles is often not the mathematically intended
     * operation and should be used with care. For example, scaling an angle is typically
     * a linear operation.
     */
    template <AxisDomain A, typename T>
    static T apply(T lhs, T rhs, OpType op) {
        T v{};
        // Stage 1: Perform the basic arithmetic operation.
        if (op == OpType::Add) {
            v = lhs + rhs;
        } else if (op == OpType::Sub) {
            // CRITICAL NOTE: For angular subtraction (e.g., finding the shortest angle
            // between two headings), this simple subtraction is only the first step.
            // The normalization below correctly wraps the result. For example,
            // 10° - 350° = -340°, which is then normalized to -20°.
            v = lhs - rhs;
        } else if (op == OpType::Mul) {
            v = lhs * rhs;
        } else {
            v = lhs / rhs;
        }

        // Stage 2: Apply normalization based on the compile-time AxisDomain.
        if constexpr (A == AxisDomain::WrappedAngular90) {
            // Normalizes the result to the range [-90, 90].
            // (e.g. clamping latitude or elevation angles)
            v = std::fmod(v + 90.0, T(180.0));
            if (v < 0) v += 180.0;
            v -= 90.0;
        } else if constexpr (A == AxisDomain::WrappedAngular180) {
            // Normalizes the result to the range [-180, 180).
            // (e.g. handles wrap-around for longitude)
            v = std::fmod(v + 180.0, T(360.0));
            if (v < 0) v += 360.0;
            v -= 180.0;
        } else if constexpr (A == AxisDomain::WrappedAngular360) {
            // Normalizes the result to the range [0, 360).
            // (e.g. used for azimuth or heading angles)
            v = std::fmod(v, T(360.0));
            if (v < 0) v += 360.0;
        }
        // For AxisDomain::Linear, no normalization is applied.
        return v;
    }
    /**
     * @brief Performs trait-aware, component-wise addition of two vectors.
     * @details For each axis, this function dispatches to the appropriate addition
     * logic based on the axis type defined in `FrameTraits<Frame>`.
     * @param a The left-hand side operand.
     * @param b The right-hand side operand.
     * @return The resulting `FramedVecType` after addition.
     */
    static FramedVecType add(const FramedVecType& a, const FramedVecType& b) {
        return {apply<FrameTraits<Frame>::axis[0]>(a.x(), b.x(), OpType::Add),
                apply<FrameTraits<Frame>::axis[1]>(a.y(), b.y(), OpType::Add),
                apply<FrameTraits<Frame>::axis[2]>(a.z(), b.z(), OpType::Add)};
    }

    /**
     * @brief Performs trait-aware, component-wise subtraction of two vectors.
     * @details This is the key function for handling non-linear frames. For an axis
     * marked as `WrappedAngular180` (like longitude), it will invoke a shortest-angle
     * subtraction. For linear axes, it performs standard subtraction.
     * @param a The minuend (the vector to subtract from).
     * @param b The subtrahend (the vector to subtract).
     * @return The resulting `FramedVecType` after subtraction.
     */
    static FramedVecType sub(const FramedVecType& a, const FramedVecType& b) {
        return {apply<FrameTraits<Frame>::axis[0]>(a.x(), b.x(), OpType::Sub),
                apply<FrameTraits<Frame>::axis[1]>(a.y(), b.y(), OpType::Sub),
                apply<FrameTraits<Frame>::axis[2]>(a.z(), b.z(), OpType::Sub)};
    }

    /**
     * @brief Performs trait-aware scalar multiplication (vector * scalar).
     * @details Applies scalar multiplication to each component. For angular types,
     * the result may be normalized to maintain a canonical representation.
     * @param a The vector operand.
     * @param b The scalar multiplier.
     * @return The scaled `FramedVecType`.
     */
    static FramedVecType scalar_mul(const FramedVecType& a, T b) {
        return {apply<FrameTraits<Frame>::axis[0]>(a.x(), b, OpType::Mul),
                apply<FrameTraits<Frame>::axis[1]>(a.y(), b, OpType::Mul),
                apply<FrameTraits<Frame>::axis[2]>(a.z(), b, OpType::Mul)};
    }

    /**
     * @brief Performs trait-aware scalar multiplication (scalar * vector).
     * @param a The scalar multiplier.
     * @param b The vector operand.
     * @return The scaled `FramedVecType`.
     */
    static FramedVecType scalar_mul(T a, const FramedVecType& b) {
        // Multiplication is commutative, so we can reuse the other implementation.
        return scalar_mul(b, a);
    }

    /**
     * @brief Performs trait-aware scalar division (vector / scalar).
     * @details Applies scalar division to each component.
     * @param a The vector dividend.
     * @param b The scalar divisor.
     * @return The resulting `FramedVecType`.
     */
    static FramedVecType scalar_div(const FramedVecType& a, T b) {
        return {apply<FrameTraits<Frame>::axis[0]>(a.x(), b, OpType::Div),
                apply<FrameTraits<Frame>::axis[1]>(a.y(), b, OpType::Div),
                apply<FrameTraits<Frame>::axis[2]>(a.z(), b, OpType::Div)};
    }

    /**
     * @brief Performs trait-aware, component-wise division (scalar / vector).
     * @details This is an element-wise operation: `result[i] = a / b[i]`.
     * @param a The scalar dividend.
     * @param b The vector divisor.
     * @return The resulting `FramedVecType`.
     */
    static FramedVecType scalar_div(T a, const FramedVecType& b) {
        // Division is commutative, so we can reuse the other implementation.
        return scalar_div(b, a);
    }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_GEOMETRY_INTERNAL_OPERATORS_ */
