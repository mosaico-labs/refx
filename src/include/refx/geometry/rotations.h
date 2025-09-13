#ifndef _REFX_GEOMETRY_ROTATIONS_
#define _REFX_GEOMETRY_ROTATIONS_

#include <cmath>
#include <ostream>

#include "../math/angles.h"
#include "internal/euler_base.h"
#include "internal/traits.h"
#include "internal/vector_base.h"
#include "vector.h"

// Conditionally include Eigen headers if support is enabled
#ifdef REFX_ENABLE_EIGEN_SUPPORT
#include <Eigen/Dense>
#endif

/**
 * @file rotations.h
 * @brief Defines the frame-aware Rotation class and related orientation structures.
 * @details This file provides the core `refx::Rotation` class, a tool for
 * representing and manipulating 3D rotations in a type-safe manner. It also defines
 * the frame-agnostic helper structs `UnitQuaternion` and `EulerAngles` to serve as
 * clean data containers for passing orientation information to and from the main
 * `Rotation` class. This architectural separation between a geometric operator
 * (`Rotation`) and its raw data representations (`UnitQuaternion`, `EulerAngles`)
 * is a key design feature that enhances clarity and type safety.
 */

#include <string_view>

namespace refx {

/**
 * @brief A simple, frame-agnostic container for a unit quaternion with algebra operators.
 * @tparam T The underlying scalar type.
 * @details This struct acts as a plain data object to hold the four components of a
 * unit quaternion (`w, x, y, z`). It has no frame safety and is intended to be used as an
 * intermediary for creating or extracting orientation data from the main `Rotation` class.
 * It also provides the fundamental operators for quaternion algebra (multiplication,
 * conjugation), which are used by the higher-level `Rotation` class.
 */
template <typename T = double>
struct UnitQuaternion : public internal::VectorContainer4D<T> {
    /// @brief Default constructor, initializes to an identity quaternion (0,0,0,1).
    UnitQuaternion() : internal::VectorContainer4D<T>(0, 0, 0, 1) {}
    /**
     * @brief Constructs a quaternion from its components.
     * @param x The x component of the vector part.
     * @param y The y component of the vector part.
     * @param z The z component of the vector part.
     * @param w The scalar part.
     */
    UnitQuaternion(T w, T x, T y, T z) : internal::VectorContainer4D<T>(x, y, z, w) {}

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Constructs a `UnitQuaternion` from an `Eigen::Quaternion`.
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * It provides the convert functionality from Eigen's quaternion type to this
     * library's data container.
     * @param eigen_quat The `Eigen::Quaternion<T>` to copy from.
     */
    explicit UnitQuaternion(const Eigen::Quaternion<T>& eigen_quat)
        : internal::VectorContainer4D<T>(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
                                         eigen_quat.w()) {}

    /**
     * @brief Constructs a `UnitQuaternion` from an `Eigen::Matrix3<T>` (rotation matrix).
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * It provides a convenient way to create a `UnitQuaternion` object from a standard
     * Direction Cosine Matrix (DCM).
     * @param rot_matrix The 3x3 Eigen rotation matrix.
     */
    explicit UnitQuaternion(const Eigen::Matrix<T, 3, 3>& rot_matrix)
        : UnitQuaternion(Eigen::Quaternion<T>(rot_matrix)) {}

#endif

    /**
     * @brief Creates a unit quaternion representing a rotation about the X-axis.
     * @param angle The angle of rotation in **radians**.
     * @return A `UnitQuaternion<T>` object representing the rotation.
     * @note This is the transpose of the DCM: if `a` is the angle 1->2 (1, 2 two rotated frames),
     * then it is such that X_1 = R(a)X_2.
     * The resulting matrix is: [1, 0, 0; 0, cA, -sA; 0, sA, cA]
     */
    static UnitQuaternion from_rotation_x(T angle);

    /**
     * @brief Creates a unit quaternion representing a rotation about the Y-axis.
     * @param angle The angle of rotation in **radians**.
     * @return A `UnitQuaternion<T>` object representing the rotation.
     * @note This is the transpose of the DCM: if `a` is the angle 1->2 (1, 2 two rotated frames),
     * then it is such that X_1 = R(a)X_2.
     * The resulting matrix is: [cA, 0, sA|; 0, 1, 0; -sA, 0, cA]
     */
    static UnitQuaternion from_rotation_y(T angle);

    /**
     * @brief Creates a unit quaternion representing a rotation about the Z-axis.
     * @param angle The angle of rotation in **radians**.
     * @return A `UnitQuaternion<T>` object representing the rotation.
     * @note This is the transpose of the DCM: if `a` is the angle 1->2 (1, 2 two rotated frames),
     * then it is such that X_1 = R(a)X_2.
     * The resulting matrix is: [cA, -sA, 0; sA, cA, 0; 0, 0, 1]
     */
    static UnitQuaternion from_rotation_z(T angle);

    // --- Quaternion Algebra ---

    /**
     * @brief Computes the conjugate of the quaternion.
     * @details For a unit quaternion, the conjugate is its inverse.
     * @return A new `UnitQuaternion` representing the conjugate.
     */
    UnitQuaternion conjugate() const {
        return UnitQuaternion(this->w(), -this->x(), -this->y(), -this->z());
    }

    /**
     * @brief Normalizes the quaternion in-place to ensure it has unit length.
     */
    void normalize();

    /**
     * @brief In-place quaternion multiplication (Hamilton product).
     * @param rhs The right-hand side quaternion.
     * @return A reference to this quaternion after multiplication.
     */
    UnitQuaternion& operator*=(const UnitQuaternion& rhs);

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Converts this `UnitQuaternion` to an `Eigen::Quaternion`.
     * @details This method is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * It provides a direct conversion to Eigen's native quaternion type for use
     * in advanced linear algebra operations.
     * @return An `Eigen::Quaternion<T>` with the same component values.
     */
    Eigen::Quaternion<T> to_eigen() const {
        return Eigen::Quaternion<T>(this->w(), this->x(), this->y(), this->z());
    }
#endif
};

/**
 * @brief Quaternion multiplication (Hamilton product).
 * @param lhs The left-hand side quaternion.
 * @param rhs The right-hand side quaternion.
 * @return A new `UnitQuaternion` representing the composition.
 */
template <typename T>
UnitQuaternion<T> operator*(UnitQuaternion<T> lhs, const UnitQuaternion<T>& rhs);

/**
 * @brief Provides a convenient stream output operator for debugging quaternions.
 * @details Formats the quaternion's components as `[w, x, y, z]`.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const UnitQuaternion<T>& q) {
    os << "[" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";
    return os;
}

/**
 * @brief Defines the sequence of *intrinsic* rotations for Tait-Bryan Euler angles.
 * @details Intrinsic rotations mean the axes move with each successive rotation.
 * For example, ZYX means: first rotate about the original Z axis, then rotate
 * about the *new* Y axis, then rotate about the *final* X axis.
 */
enum class EulerSequence {
    ZYX,  ///< Standard for aerospace and navigation (Yaw, Pitch, Roll).
    ZXY,  ///< e.g., for camera pointing.
    YZX,  ///< A valid Tait-Bryan sequence.
    YXZ,  ///< e.g., for character animation.
    XYZ,  ///< Common in robotics and computer graphics.
    XZY   ///< A valid Tait-Bryan sequence.
};

/**
 * @brief A type-safe, frame-agnostic container for Euler angles with an explicit sequence.
 * @tparam Seq The `EulerSequence` defining the order of rotations.
 * @tparam T The underlying scalar type.
 * @details This struct acts as a plain data object to hold the three Euler angles.
 * By templating on the sequence, it prevents ambiguity about the meaning of the
 * angle values. The constructor arguments are ordered according to the sequence.
 *
 * For example, the specialization for `EulerSequence::YXZ` will expect its
 * constructor arguments to be `(angle_y, angle_x, angle_z)`.
 * All angles are stored and handled in **radians**.
 */
template <EulerSequence Seq, typename T = double>
struct EulerAngles;  // This generic template is specialized below for each sequence.

enum class AxisOrder : size_t { Axis0 = 0, Axis1, Axis2 };

/**
 * @brief Specialization for the **ZYX (Yaw, Pitch, Roll)** sequence.
 * @details This is the most common sequence in aerospace and navigation,
 * used to compute the rotation matrix for rotating a vector in body frame
 * to navigation frame.
 * - `yaw`: First rotation in **radians**, about the Z-axis.
 * - `pitch`: Second rotation in **radians**, about the new Y-axis.
 * - `roll`: Third rotation in **radians**, about the final X-axis.
 */
template <typename T>
struct EulerAngles<EulerSequence::ZYX, T> : public internal::EulerBase<T> {
    /// @brief Defines the angle range of each angle
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngular180,  ///< Roll wraps around [-180°, 180°].
        AxisDomain::WrappedAngular90,   ///< Pitch wraps around at [-90°, 90°].
        AxisDomain::WrappedAngular180   ///< Yaw wraps around [-180°, 180°].
    };
    /// @brief Defines the axis rotation order
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis2,  ///< First rotation around the z-axis.
        AxisOrder::Axis1,  ///< Second rotation around the y-axis.
        AxisOrder::Axis0,  ///< Third rotation around the x-axis.
    };
    EulerAngles() : internal::EulerBase<T>(0, 0, 0) {}
    /**
     * @brief Constructs a ZYX Euler angle object.
     * @param yaw The first rotation, about the Z-axis, in **radians**.
     * @param pitch The second rotation, about the new Y-axis, in **radians**.
     * @param roll The third rotation, about the final X-axis, in **radians**.
     */
    EulerAngles(T yaw, T pitch, T roll) : internal::EulerBase<T>(roll, pitch, yaw) {}

    // --- Named Accessors ---
    T& roll() { return this->angle_x(); }
    T& pitch() { return this->angle_y(); }
    T& yaw() { return this->angle_z(); }

    const T& roll() const { return this->angle_x(); }
    const T& pitch() const { return this->angle_y(); }
    const T& yaw() const { return this->angle_z(); }

    EulerAngles<EulerSequence::XYZ, T> inverse() {
        return EulerAngles<EulerSequence::XYZ>(-this->angle_x(), -this->angle_y(),
                                               -this->angle_z());
    }
};

/// @brief A convenience alias for the common ZYX Euler angle sequence.
/// All angles are stored and handled in **radians**.
template <typename T>
using YawPitchRoll = EulerAngles<EulerSequence::ZYX, T>;

/// @brief Specialization for the ZXY Euler angle sequence.
/// All angles are stored and handled in **radians**.
template <typename T>
struct EulerAngles<EulerSequence::ZXY, T> : public internal::EulerBase<T> {
    /// @brief Defines the angle range of each angle
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngular90,   ///< ang_x (2nd angle) wraps around at [-90°, 90°].
        AxisDomain::WrappedAngular180,  ///< ang_y (3rd angle) wraps around [-180°, 180°].
        AxisDomain::WrappedAngular180   ///< ang_z (1st angle) wraps around [-180°, 180°].
    };
    /// @brief Defines the axis rotation order
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis2,  ///< First rotation around the z-axis.
        AxisOrder::Axis0,  ///< Second rotation around the x-axis.
        AxisOrder::Axis1,  ///< Third rotation around the y-axis.
    };
    EulerAngles() : internal::EulerBase<T>(0, 0, 0) {}
    EulerAngles(T ang_z, T ang_x, T ang_y) : internal::EulerBase<T>(ang_x, ang_y, ang_z) {}
    EulerAngles<EulerSequence::YXZ, T> inverse() {
        return EulerAngles<EulerSequence::YXZ>(-this->angle_y(), -this->angle_x(),
                                               -this->angle_z());
    }
};

/// @brief Specialization for the YZX Euler angle sequence.
/// All angles are stored and handled in **radians**.
template <typename T>
struct EulerAngles<EulerSequence::YZX, T> : public internal::EulerBase<T> {
    /// @brief Defines the angle range of each angle
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngular180,  ///< ang_x (3rd angle) wraps around at [-90°, 90°].
        AxisDomain::WrappedAngular180,  ///< ang_y (1st angle) wraps around [-180°, 180°].
        AxisDomain::WrappedAngular90    ///< ang_z (2nd angle) wraps around [-180°, 180°].
    };
    /// @brief Defines the axis rotation order
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis1,  ///< First rotation around the y-axis.
        AxisOrder::Axis2,  ///< Second rotation around the z-axis.
        AxisOrder::Axis0,  ///< Third rotation around the x-axis.
    };
    EulerAngles() : internal::EulerBase<T>(0, 0, 0) {}
    EulerAngles(T ang_y, T ang_z, T ang_x) : internal::EulerBase<T>(ang_x, ang_y, ang_z) {}
    EulerAngles<EulerSequence::XZY, T> inverse() {
        return EulerAngles<EulerSequence::XZY>(-this->angle_x(), -this->angle_z(),
                                               -this->angle_y());
    }
};

/// @brief Specialization for the YXZ Euler angle sequence.
/// All angles are stored and handled in **radians**.
template <typename T>
struct EulerAngles<EulerSequence::YXZ, T> : public internal::EulerBase<T> {
    /// @brief Defines the angle range of each angle
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngular90,   ///< ang_x (2nd angle) wraps around at [-90°, 90°].
        AxisDomain::WrappedAngular180,  ///< ang_y (1st angle) wraps around [-180°, 180°].
        AxisDomain::WrappedAngular180   ///< ang_z (3rd angle) wraps around [-180°, 180°].
    };
    /// @brief Defines the axis rotation order
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis1,  ///< First rotation around the y-axis.
        AxisOrder::Axis0,  ///< Second rotation around the x-axis.
        AxisOrder::Axis2,  ///< Third rotation around the z-axis.
    };

    EulerAngles() : internal::EulerBase<T>(0, 0, 0) {}
    EulerAngles(T ang_y, T ang_x, T ang_z) : internal::EulerBase<T>(ang_x, ang_y, ang_z) {}
    EulerAngles<EulerSequence::ZXY, T> inverse() {
        return EulerAngles<EulerSequence::ZXY>(-this->angle_z(), -this->angle_x(),
                                               -this->angle_y());
    }
};

/// @brief Specialization for the XZY Euler angle sequence.
/// All angles are stored and handled in **radians**.
template <typename T>
struct EulerAngles<EulerSequence::XZY, T> : public internal::EulerBase<T> {
    /// @brief Defines the angle range of each angle
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngular180,  ///< ang_x (1st angle) wraps around at [-90°, 90°].
        AxisDomain::WrappedAngular180,  ///< ang_y (3rd angle) wraps around [-180°, 180°].
        AxisDomain::WrappedAngular90    ///< ang_z (2nd angle) wraps around [-180°, 180°].
    };
    /// @brief Defines the axis rotation order
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis0,  ///< First rotation around the x-axis.
        AxisOrder::Axis2,  ///< Second rotation around the z-axis.
        AxisOrder::Axis1,  ///< Third rotation around the y-axis.
    };

    EulerAngles() : internal::EulerBase<T>(0, 0, 0) {}
    EulerAngles(T ang_x, T ang_z, T ang_y) : internal::EulerBase<T>(ang_x, ang_y, ang_z) {}
    EulerAngles<EulerSequence::YZX, T> inverse() {
        return EulerAngles<EulerSequence::YZX>(-this->angle_y(), -this->angle_z(),
                                               -this->angle_x());
    }
};

/// @brief Specialization for the XYZ Euler angle sequence.
/// All angles are stored and handled in **radians**.
template <typename T>
struct EulerAngles<EulerSequence::XYZ, T> : public internal::EulerBase<T> {
    /// @brief Defines the angle range of each angle
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngular180,  ///< ang_x (1st angle) wraps around at [-90°, 90°].
        AxisDomain::WrappedAngular90,   ///< ang_y (2nd angle) wraps around [-180°, 180°].
        AxisDomain::WrappedAngular180   ///< ang_z (3rd angle) wraps around [-180°, 180°].
    };
    /// @brief Defines the axis rotation order
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis0,  ///< First rotation around the x-axis.
        AxisOrder::Axis1,  ///< Second rotation around the y-axis.
        AxisOrder::Axis2,  ///< Third rotation around the z-axis.
    };

    EulerAngles() : internal::EulerBase<T>(0, 0, 0) {}
    EulerAngles(T ang_x, T ang_y, T ang_z) : internal::EulerBase<T>(ang_x, ang_y, ang_z) {}
    EulerAngles<EulerSequence::ZYX, T> inverse() {
        return EulerAngles<EulerSequence::ZYX>(-this->angle_z(), -this->angle_y(),
                                               -this->angle_x());
    }
};

// Forward declaration for a compile-time helper function
namespace internal {
template <EulerSequence Seq>
constexpr std::string_view get_sequence_name();
}

/**
 * @brief Provides a convenient stream output operator for debugging euler angles.
 * @details Formats the euler angles components as `Seq[angle_0, angle_1, angle_2]`.
 */
template <EulerSequence Seq, typename T>
std::ostream& operator<<(std::ostream& os, const EulerAngles<Seq, T>& eul) {
    // Print the sequence name for an unambiguous representation.
    os << internal::get_sequence_name<Seq>();

    // Use the `axis_order` array as a lookup table to print the angles
    // in their correct semantic sequence.
    os << "[" << eul.data()[static_cast<size_t>(EulerAngles<Seq, T>::axis_order[0])] << ", "
       << eul.data()[static_cast<size_t>(EulerAngles<Seq, T>::axis_order[1])] << ", "
       << eul.data()[static_cast<size_t>(EulerAngles<Seq, T>::axis_order[2])] << "]";
    return os;
}

/**
 * @brief A type-safe representation of a 3D rotation between two reference frames.
 * @tparam ToFrame The destination (or parent) frame of the rotation.
 * @tparam FromFrame The source (or child) frame of the rotation.
 * @tparam T The underlying scalar type (e.g., `float`, `double`).
 *
 * @details This class represents the orientation of `FromFrame` with respect to
 * `ToFrame`. It is the primary geometric operator for handling rotations. It is internally
 * represented by a unit quaternion for efficiency and numerical stability.
 *
 * The key feature of this class is its compile-time frame safety. The multiplication
 * operators are overloaded to enforce mathematically correct chaining of rotations
 * and transformations of vectors.
 */
template <typename ToFrame, typename FromFrame, typename T = double>
struct Rotation {
    static_assert(is_valid_rotation_v<ToFrame, FromFrame>,
                  "Rotation operator is only allowed for DirectionalAxis (Cartesian) frames");

   private:
    UnitQuaternion<T> m_quat;  ///< Internal quaternion storage.

   public:
    /**
     * @brief Default constructor. Creates an identity rotation (no rotation).
     */
    Rotation() = default;

    /**
     * @brief Constructs a rotation from a frame-agnostic `UnitQuaternion`.
     * @warning The user is responsible for ensuring the input quaternion is normalized.
     * @param q The `UnitQuaternion` representing the desired orientation.
     */
    explicit Rotation(const UnitQuaternion<T>& q) : m_quat(q) {}

    /**
     * @brief Constructs a rotation from a type-safe `EulerAngles` object.
     * @tparam Seq The `EulerSequence` of the input angles.
     * @param angles An `EulerAngles<Seq, T>` struct.
     */
    template <EulerSequence Seq>
    explicit Rotation(const EulerAngles<Seq, T>& angles);

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Constructs a `Rotation` from an `Eigen::Quaternion`.
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * @warning The user is responsible for ensuring the input Eigen quaternion is normalized.
     * @param eigen_quat The `Eigen::Quaternion<T>` to construct from.
     */
    explicit Rotation(const Eigen::Quaternion<T>& eigen_quat) : m_quat(eigen_quat) {}

    /**
     * @brief Constructs a `Rotation` from an `Eigen::Matrix3<T>` (rotation matrix).
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * It provides a convenient way to create a `Rotation` object from a standard
     * Direction Cosine Matrix (DCM).
     * @param rot_matrix The 3x3 Eigen rotation matrix.
     */
    explicit Rotation(const Eigen::Matrix<T, 3, 3>& rot_matrix)
        : m_quat(UnitQuaternion<T>(rot_matrix)) {}
#endif /* _REFX_GEOMETRY_ROTATIONS_ */

    // --- Factory Functions ---
    /**
     * @brief Creates a rotation from an axis-angle representation.
     * The axis-angle rotation representation vector is expressed relative
     * to the current reference frame.
     * @param axis The axis of rotation. Must be a unit vector.
     * @param angle The angle of rotation about the axis, in **radians**.
     * @return A `Rotation` object.
     */
    static Rotation from_axis_angle(const Vector3D<FromFrame, T>& axis, T angle);

    // --- Conversion and Access ---

    /**
     * @brief Extracts the internal orientation as a frame-agnostic `UnitQuaternion`.
     * @return A `UnitQuaternion<T>` data container.
     */
    UnitQuaternion<T> to_quaternion() const { return m_quat; }

    /**
     * @brief Converts the internal orientation to a type-safe `EulerAngles` object.
     * @tparam Seq The desired `EulerSequence` for the output angles.
     * @return An `EulerAngles<Seq, T>` struct with angles in **radians**.
     */
    template <EulerSequence Seq>
    EulerAngles<Seq, T> to_euler_angles() const;

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Converts this `Rotation` to an `Eigen::Quaternion`.
     * @details This method is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * @return An `Eigen::Quaternion<T>` representing the rotation.
     */
    Eigen::Quaternion<T> to_eigen_quat() const { return m_quat.to_eigen(); }

    /**
     * @brief Converts this `Rotation` to an `Eigen::Matrix3<T>` (rotation matrix).
     * @details This method is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * @return A 3x3 Eigen rotation matrix (DCM).
     */
    Eigen::Matrix<T, 3, 3> to_eigen_matrix() const { return m_quat.to_eigen().toRotationMatrix(); }
#endif /* REFX_ENABLE_EIGEN_SUPPORT */

    // --- Core Operations ---

    /**
     * @brief Computes the inverse of the rotation.
     * @details For a rotation from frame B to A, the inverse represents the rotation
     * from frame A to B. For a unit quaternion, this is simply its conjugate.
     * @return A `Rotation<FromFrame, ToFrame, T>` representing the inverse rotation.
     */
    Rotation<FromFrame, ToFrame, T> inverse() const {
        return Rotation<FromFrame, ToFrame, T>(m_quat.conjugate());
    }

    /**
     * @brief Normalizes the internal quaternion to ensure it has unit length.
     */
    void normalize() { m_quat.normalize(); }
};

/**
 * @brief Provides a convenient stream output operator for debugging rotations.
 * @details Prints the rotation quaternion as `[w, x, y, z]`.
 */
template <typename ToFrame, typename FromFrame, typename T = double>
std::ostream& operator<<(std::ostream& os, const Rotation<ToFrame, FromFrame, T>& rot) {
    os << rot.to_quaternion();
    return os;
}

// --- Free-conversion functions ---
/**
 * @brief Constructs a EulerAngles<Seq, T> from a UnitQuaternion<T>
 */
template <EulerSequence Seq, typename T>
EulerAngles<Seq, T> quat_to_euler(const UnitQuaternion<T>& quat);

/**
 * @brief Constructs a UnitQuaternion<T> from a EulerAngles<Seq, T>
 */
template <EulerSequence Seq, typename T>
UnitQuaternion<T> euler_to_quat(const EulerAngles<Seq, T>& eul);

// --- Compile-Time Safe Operators ---

/**
 * @brief Rotates a vector/coordinate from the source frame to the destination frame.
 */
template <typename FrameA, typename FrameB, template <class, class> class VecType, typename T>
VecType<FrameA, T> operator*(const Rotation<FrameA, FrameB, T>& R, const VecType<FrameB, T>& v);

/**
 * @brief Composes two rotations. `R_ac = R_ab * R_bc`
 */
template <typename FrameA, typename FrameB, typename FrameC, typename T>
Rotation<FrameA, FrameC, T> operator*(const Rotation<FrameA, FrameB, T>& R_ab,
                                      const Rotation<FrameB, FrameC, T>& R_bc);

}  // namespace refx

#include "internal/rotations.hpp"

#endif /* _REFX_GEOMETRY_ROTATIONS_ */
