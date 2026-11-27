#ifndef _REFX_GEOMETRY_TRANSFORMATION_
#define _REFX_GEOMETRY_TRANSFORMATION_

#include "../internal/always_false.h"
#include "coordinate.h"
#include "internal/traits.h"
#include "rotations.h"
#include "vector.h"

namespace refx {

/**
 * @file transformation.h
 * @brief Represents a rigid body transformation (pose) in 3D space.
 * @details This struct models an element of the Special Euclidean group SE(3),
 * which is the standard mathematical representation for the pose (position and
 * orientation) of an object in 3D. It combines a rotation and a translation
 * into a single, cohesive object that defines a complete coordinate frame
 * transformation from a source frame (`FromFrame`) to a destination frame (`ToFrame`).
 *
 * The transformation of a point P from the `FromFrame` to the `ToFrame` is
 * defined by the operation:
 * `P_in_ToFrame = rotation * P_in_FromFrame + translation`
 *
 * This struct is fundamental for tasks like transforming sensor measurements
 * into a global frame, calculating the world position of a robot's end-effector,
 * or chaining multiple coordinate system changes using the composition operator (*).
 *
 * @tparam ToFrame The destination (target) coordinate frame tag.
 * @tparam FromFrame The source coordinate frame tag.
 * @tparam T The underlying scalar type for calculations (e.g., double, float).
 */
template <typename ToFrame, typename FromFrame, typename T = double>
struct Transformation {
    static_assert(is_valid_transformation_v<ToFrame, FromFrame>,
                  "Transformation operator is only allowed for DirectionalAxis (Cartesian) frames");

    Transformation()
        : rotation(Rotation<ToFrame, FromFrame, T>()), translation(Vector3D<ToFrame, T>()) {}

    /// @brief Constructs a Transformation from a Vector3D.
    Transformation(const Rotation<ToFrame, FromFrame, T>& r, const Vector3D<ToFrame, T>& t)
        : rotation(r), translation(t) {}

    /// @brief Constructs a Transformation from a Coordinate3D.
    Transformation(const Rotation<ToFrame, FromFrame, T>& r, const Coordinate3D<ToFrame, T>& t)
        : rotation(r), translation(t.as_vector()) {}

    /**
     * @brief The rotational component of the transformation.
     *
     * This object represents the orientation of the `FromFrame` relative
     * to the `ToFrame`.
     */
    Rotation<ToFrame, FromFrame, T> rotation;

    /**
     * @brief The translational component of the transformation.
     *
     * This vector represents the displacement of the `FromFrame`'s origin
     * as measured from the `ToFrame`'s origin, and expressed in the
     * coordinates of the `ToFrame`.
     */
    Vector3D<ToFrame, T> translation;

    /**
     * @brief Computes the inverse of the transformation.
     *
     * @details If this transformation `T_To_From` converts points from `FromFrame`
     * to `ToFrame`, the inverse transformation converts points back from `ToFrame`
     * to `FromFrame`. This is useful for changing the direction of a known
     * relationship between two coordinate systems.
     *
     * The mathematical formula for the inverse is:
     * - `rotation_inv = rotation.inverse()`
     * - `translation_inv = -(rotation.inverse() * translation)`
     *
     * @return A new Transformation object of type `Transformation<FromFrame, ToFrame, T>`
     * representing the inverse transformation.
     */
    Transformation<FromFrame, ToFrame, T> inverse() const {
        Rotation<FromFrame, ToFrame, T> inv_rotation = rotation.inverse();
        Vector3D<FromFrame, T> inv_translation = -(inv_rotation * translation);
        return {inv_rotation, inv_translation};
    }
};

/**
 * @brief Applies a pose to a point (coordinate): `p_A = R_A_B * p_B + t_A_B`.
 * @details A coordinate is a position, so it is both rotated and translated.
 * The compile-time frame safety ensures that the operation can be made only if the input
 * coordinate matches the `FromFrame` of the Transformation.
 *
 * @tparam FrameA The destination frame.
 * @tparam FrameB The source frame of the transformation.
 * @tparam FrameC The frame of the coordinate (must be equal to `FrameB`).
 * @tparam T The underlying scalar type.
 * @param T_A_B The transformation from frame B to frame A.
 * @param p The coordinate expressed in frame B.
 * @return The same point, expressed in frame A.
 */
template <typename FrameA, typename FrameB, typename FrameC, typename T>
Coordinate3D<FrameA, T> operator*(const Transformation<FrameA, FrameB, T>& T_A_B,
                                  const Coordinate3D<FrameC, T>& p) {
    internal::FrameValidator<FrameB, FrameC>::validate();
    return T_A_B.rotation * p + T_A_B.translation;
}

/**
 * @brief Applying a pose to a free vector is disallowed (compile-time error).
 * @details A vector (displacement, velocity, acceleration, force, ...) does not depend on
 * the origin of the frame: under a change of frame it is only rotated, the translation of the
 * pose does not apply (for a displacement `d = p2 - p1`: `T * p2 - T * p1 = R * d`).
 * To avoid any ambiguity, the operation must be written explicitly:
 * - `T_A_B.rotation * v` to re-express a vector in frame A;
 * - `T_A_B * p` with `p` a `Coordinate3D`, to transform a position.
 */
template <typename FrameA, typename FrameB, typename FrameC, typename T>
Vector3D<FrameA, T> operator*(const Transformation<FrameA, FrameB, T>&,
                              const Vector3D<FrameC, T>&) {
    static_assert(internal::always_false_v<FrameA, FrameB, FrameC, T>,
                  "A Transformation cannot be applied to a Vector3D: a vector is only rotated by a "
                  "change of frame. Use `T.rotation * v` for vectors, or apply the Transformation "
                  "to a Coordinate3D to transform a position.");
    return {};
}

/**
 * @brief Implements the composition (chaining) of two Transformations.
 * @details This operator allows for chaining transformations in a natural and
 * type-safe way. For example, if you have the transform from a camera to a
 * vehicle (`T_B_C`) and the transform from the vehicle to the world (`T_A_B`),
 * this operator computes the direct transform from the camera to the world (`T_A_C`).
 *
 * The compile-time frame safety ensures that transformations can only be
 * composed if the `ToFrame` of the right-hand side matches the `FromFrame` of the
 * left-hand side.
 *
 * @tparam FrameA The final destination frame (e.g., world).
 * @tparam FrameB The intermediate frame (e.g., vehicle).
 * @tparam FrameC The original source frame (e.g., camera).
 * @tparam T The underlying scalar type.
 * @param T_A_B The transformation from frame B to frame A.
 * @param T_B_C The transformation from frame C to frame B.
 * @return A new Transformation object of type `Transformation<A, C, T>` representing
 * the direct transformation from frame C to frame A.
 */
template <typename FrameA, typename FrameB, typename FrameC, typename T>
Transformation<FrameA, FrameC, T> operator*(const Transformation<FrameA, FrameB, T>& T_A_B,
                                            const Transformation<FrameB, FrameC, T>& T_B_C) {
    return {T_A_B.rotation * T_B_C.rotation,
            T_A_B.rotation * T_B_C.translation + T_A_B.translation};
}

/**
 * @brief Provides a convenient stream output operator for debugging transformations.
 * @details Formats the transformation's components as `{[qw, qx, qy, qz]; [tx, ty, tz]}`.
 */
template <typename ToFrame, typename FromFrame, typename T = double>
std::ostream& operator<<(std::ostream& os, const Transformation<ToFrame, FromFrame, T>& t) {
    os << "{" << t.rotation << "; " << t.translation << "}";
    return os;
}
}  // namespace refx

#endif /* _REFX_GEOMETRY_TRANSFORMATION_ */
