#ifndef _REFX_GEOMETRY_INTERNAL_ROTATIONS_
#define _REFX_GEOMETRY_INTERNAL_ROTATIONS_

#include <algorithm>
#include <string_view>

#include "../internal/operators.h"
#include "../rotations.h"

/**
 * @file rotations.hpp
 * @brief Provides the out-of-class implementations for the Rotation class and its operators.
 * @details This is an internal implementation header and is not intended for direct
 * inclusion by end-users. It contains the mathematical logic for converting between
 * different orientation representations (Euler angles, axis-angle, quaternions) and
 * the implementation of the type-safe operators for rotation composition and vector
 * transformation.
 *
 * @warning This file should only be included by `rotations.h`.
 */

namespace refx {

namespace internal {
/**
 * @brief Compile-time helper to get the string representation of an EulerSequence.
 * @tparam Seq The EulerSequence value.
 * @return A std::string_view of the sequence name (e.g., "ZYX").
 */
template <EulerSequence Seq>
constexpr std::string_view get_sequence_name() {
    if constexpr (Seq == EulerSequence::ZYX) return "ZYX";
    if constexpr (Seq == EulerSequence::ZXY) return "ZXY";
    if constexpr (Seq == EulerSequence::YZX) return "YZX";
    if constexpr (Seq == EulerSequence::YXZ) return "YXZ";
    if constexpr (Seq == EulerSequence::XYZ) return "XYZ";
    if constexpr (Seq == EulerSequence::XZY) return "XZY";
    return "Unknown";
}
}  // namespace internal

//==========================================================================//
//================ EulerAngles METHOD IMPLEMENTATIONS ===================//
//==========================================================================//

template <EulerSequence Seq, typename T>
UnitQuaternion<T> euler_to_quat(const EulerAngles<Seq, T>& eul) {
    // Create elementary rotation quaternions from the generic angle accessors.
    const auto& q_x = UnitQuaternion<T>::from_rotation_x(eul.angle_x());
    const auto& q_y = UnitQuaternion<T>::from_rotation_y(eul.angle_y());
    const auto& q_z = UnitQuaternion<T>::from_rotation_z(eul.angle_z());

    // Use if  to select the correct composition order at compile-time.
    // NOTE: The DCM matrix used in the navigation literature, for rotating from body to navigation
    // frame is computed as (the transposed are with respect to our definition of qrotation_x/y/z)
    // DCM = [Rx^T Ry^T Rz^T]^T = [Rz Ry Rx], that is the ZYX sequence is the body-to-nav DCM
    if constexpr (Seq == EulerSequence::ZYX) {
        return q_z * q_y * q_x;
    } else if constexpr (Seq == EulerSequence::ZXY) {
        return q_z * q_x * q_y;
    } else if constexpr (Seq == EulerSequence::YZX) {
        return q_y * q_z * q_x;
    } else if constexpr (Seq == EulerSequence::YXZ) {
        return q_y * q_x * q_z;
    } else if constexpr (Seq == EulerSequence::XYZ) {
        return q_x * q_y * q_z;
    } else if constexpr (Seq == EulerSequence::XZY) {
        return q_x * q_z * q_y;
    } else {
        static_assert(false, "Unsupported Euler sequence");
    }
}

template <EulerSequence Seq, typename T>
EulerAngles<Seq, T> quat_to_euler(const UnitQuaternion<T>& quat) {
    T qx = quat.x(), qy = quat.y(), qz = quat.z(), qw = quat.w();

    // Use if  to select the correct conversion formula at compile-time.
    if constexpr (Seq == EulerSequence::ZYX) {
        // Roll (x-axis rotation)
        T sinr_cosp = 2 * (qw * qx + qy * qz);
        T cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        T roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        T sinp = 2 * (qw * qy - qz * qx);
        T pitch = (std::abs(sinp) >= 1) ? std::copysign(M_PI_2, sinp) : std::asin(sinp);

        // Yaw (z-axis rotation)
        T siny_cosp = 2 * (qw * qz + qx * qy);
        T cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        T yaw = std::atan2(siny_cosp, cosy_cosp);
        return EulerAngles<Seq, T>(yaw, pitch, roll);
    } else if constexpr (Seq == EulerSequence::ZXY) {
        // Roll (x-axis rotation)
        T sin_r = 2.0 * (qw * qx + qy * qz);
        T angle_x, angle_y, angle_z;
        // Check for gimbal lock
        if ((1.0 - std::abs(sin_r)) < std::numeric_limits<T>::epsilon()) {
            // Use 90 degrees if locked
            angle_x = std::copysign(M_PI_2, sin_r);
            // Yaw and pitch are ambiguous. Conventionally, set pitch to 0.
            angle_y = 0.0;
            // Calculate yaw from the remaining rotation
            angle_z = std::atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
        } else {
            // No gimbal lock
            angle_x = std::asin(sin_r);
            // Pitch (y-axis rotation)
            angle_y = std::atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            // Yaw (z-axis rotation)
            angle_z = std::atan2(2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qx * qx + qz * qz));
        }
        return EulerAngles<Seq, T>(angle_z, angle_x, angle_y);

    } else if constexpr (Seq == EulerSequence::XYZ) {
        // Pitch (y-axis rotation)
        T sin_p = 2.0 * (qw * qy + qx * qz);
        T angle_x, angle_y, angle_z;
        // Check for gimbal lock
        if ((1.0 - std::abs(sin_p)) < std::numeric_limits<T>::epsilon()) {
            // Use 90 degrees if locked
            angle_y = std::copysign(M_PI_2, sin_p);
            // Roll and yaw are ambiguous. Conventionally, set yaw to 0.
            angle_z = 0.0;
            // Calculate roll from the remaining rotation
            angle_x = std::atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz));
        } else {
            // No gimbal lock
            angle_y = std::asin(sin_p);
            // Roll (x-axis rotation)
            angle_x = std::atan2(2.0 * (qw * qx - qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            // Yaw (z-axis rotation)
            angle_z = std::atan2(2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        }
        return EulerAngles<Seq, T>(angle_x, angle_y, angle_z);
    } else if constexpr (Seq == EulerSequence::XZY) {
        // Yaw (z-axis rotation)
        T sin_y = 2.0 * (qw * qz - qx * qy);
        T angle_x, angle_y, angle_z;
        // Check for gimbal lock
        if ((1.0 - std::abs(sin_y)) < std::numeric_limits<T>::epsilon()) {
            // Use 90 degrees if locked
            angle_z = std::copysign(M_PI_2, sin_y);
            // Roll and pitch are ambiguous. Conventionally, set pitch to 0.
            angle_y = 0.0;
            // Calculate roll from the remaining rotation
            angle_x = std::atan2(2.0 * (qx * qz - qw * qy), 2.0 * (qx * qy + qw * qz));

        } else {
            // No gimbal lock
            angle_z = std::asin(sin_y);
            // Roll (x-axis rotation)
            angle_x = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qz * qz));
            // Pitch (y-axis rotation)
            angle_y = std::atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
        }

        return EulerAngles<Seq, T>(angle_x, angle_z, angle_y);
    } else if constexpr (Seq == EulerSequence::YXZ) {
        // Roll (x-axis rotation)
        T sin_r = 2.0 * (qw * qx - qy * qz);
        T angle_x, angle_y, angle_z;
        // Check for gimbal lock
        if ((1.0 - std::abs(sin_r)) < std::numeric_limits<T>::epsilon()) {
            // Use 90 degrees if locked
            angle_x = std::copysign(M_PI_2, sin_r);
            // Pitch and yaw are ambiguous. Conventionally, set yaw to 0.
            angle_z = 0.0;
            // Calculate pitch from the remaining rotation
            angle_y = 2.0 * std::atan2(qy, qw);

        } else {
            // No gimbal lock
            angle_x = std::asin(sin_r);
            // Pitch (y-axis rotation)
            angle_y = std::atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            // Yaw (z-axis rotation)
            angle_z = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qx * qx + qz * qz));
        }

        return EulerAngles<Seq, T>(angle_y, angle_x, angle_z);
    } else if constexpr (Seq == EulerSequence::YZX) {
        // Yaw (z-axis rotation)
        T sin_y = 2.0 * (qw * qz + qx * qy);
        T angle_x, angle_y, angle_z;
        // Check for gimbal lock
        if ((1.0 - std::abs(sin_y)) < std::numeric_limits<T>::epsilon()) {
            // Use 90 degrees if locked
            angle_z = std::copysign(M_PI_2, sin_y);
            // Pitch and roll are ambiguous. Conventionally, set roll to 0.
            angle_x = 0.0;
            // Calculate pitch from the remaining rotation
            angle_y = 2.0 * std::atan2(qy, qw);

        } else {
            // No gimbal lock
            angle_z = std::asin(sin_y);
            // Pitch (y-axis rotation)
            angle_y = std::atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
            // Roll (x-axis rotation)
            angle_x = std::atan2(2.0 * (qw * qx - qy * qz), 1.0 - 2.0 * (qx * qx + qz * qz));
        }

        return EulerAngles<Seq, T>(angle_y, angle_z, angle_x);
    } else {
        static_assert(false, "Unsupported Euler sequence");
    }
}

/**
 * @brief Creates a unit quaternion representing an elementary rotation about the X-axis.
 * @param angle The rotation angle in **radians**.
 * @return A `UnitQuaternion` for the specified rotation.
 */
template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::from_rotation_x(T angle) {
    return UnitQuaternion<T>(std::cos(angle / 2.0), std::sin(angle / 2.0), 0.0, 0.0);
}

/**
 * @brief Creates a unit quaternion representing an elementary rotation about the Y-axis.
 * @param angle The rotation angle in **radians**.
 * @return A `UnitQuaternion` for the specified rotation.
 */
template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::from_rotation_y(T angle) {
    return UnitQuaternion<T>(std::cos(angle / 2.0), 0.0, std::sin(angle / 2.0), 0.0);
}

/**
 * @brief Creates a unit quaternion representing an elementary rotation about the Z-axis.
 * @param angle The rotation angle in **radians**.
 * @return A `UnitQuaternion` for the specified rotation.
 */
template <typename T>
UnitQuaternion<T> UnitQuaternion<T>::from_rotation_z(T angle) {
    return UnitQuaternion<T>(std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0));
}

//==========================================================================//
//================ UNITQUATERNION METHOD IMPLEMENTATIONS ===================//
//==========================================================================//

template <typename T>
void UnitQuaternion<T>::normalize() {
    T norm = this->norm();
    if (norm > 1e-9) {
        this->x() /= norm;
        this->y() /= norm;
        this->z() /= norm;
        this->w() /= norm;
    }
}

template <typename T>
UnitQuaternion<T>& UnitQuaternion<T>::operator*=(const UnitQuaternion<T>& rhs) {
    T w_new = this->w() * rhs.w() - this->x() * rhs.x() - this->y() * rhs.y() - this->z() * rhs.z();
    T x_new = this->w() * rhs.x() + this->x() * rhs.w() + this->y() * rhs.z() - this->z() * rhs.y();
    T y_new = this->w() * rhs.y() - this->x() * rhs.z() + this->y() * rhs.w() + this->z() * rhs.x();
    T z_new = this->w() * rhs.z() + this->x() * rhs.y() - this->y() * rhs.x() + this->z() * rhs.w();
    this->w() = w_new;
    this->x() = x_new;
    this->y() = y_new;
    this->z() = z_new;
    return *this;
}

template <typename T>
UnitQuaternion<T> operator*(UnitQuaternion<T> lhs, const UnitQuaternion<T>& rhs) {
    lhs *= rhs;
    return lhs;
}

//==========================================================================//
//================== ROTATION METHOD IMPLEMENTATIONS =======================//
//==========================================================================//

template <typename ToFrame, typename FromFrame, typename T>
template <EulerSequence Seq>
Rotation<ToFrame, FromFrame, T>::Rotation(const EulerAngles<Seq, T>& angles)
    : m_quat(euler_to_quat(angles)) {}

template <typename ToFrame, typename FromFrame, typename T>
template <EulerSequence Seq>
EulerAngles<Seq, T> Rotation<ToFrame, FromFrame, T>::to_euler_angles() const {
    return quat_to_euler<Seq, T>(m_quat);
}

template <typename ToFrame, typename FromFrame, typename T>
Rotation<ToFrame, FromFrame, T> Rotation<ToFrame, FromFrame, T>::from_axis_angle(
    const Vector3D<FromFrame, T>& axis, T angle) {
    T norm = axis.norm();
    if (norm < 1e-9) return Rotation();  // Return identity for zero axis
    T half_angle = angle * 0.5;
    T sin_half_angle = std::sin(half_angle);
    T w = std::cos(half_angle);
    T x = axis.x() / norm * sin_half_angle;
    T y = axis.y() / norm * sin_half_angle;
    T z = axis.z() / norm * sin_half_angle;
    return Rotation(UnitQuaternion<T>(w, x, y, z));
}

//==========================================================================//
//================== COMPILE-TIME SAFE OPERATORS ===========================//
//==========================================================================//

template <typename FrameA, typename FrameB, template <class, class> class VecType, typename T>
VecType<FrameA, T> operator*(const Rotation<FrameA, FrameB, T>& R, const VecType<FrameB, T>& v) {
    // This function implements the efficient quaternion-vector multiplication: v' = qvq*
    // where v is a pure quaternion (0, vx, vy, vz).
    UnitQuaternion<T> q = R.to_quaternion();
    T qx = q.x(), qy = q.y(), qz = q.z(), qw = q.w();
    T vx = v.x(), vy = v.y(), vz = v.z();

    // t = 2 * cross(q.xyz, v)
    T tx = 2 * (qy * vz - qz * vy);
    T ty = 2 * (qz * vx - qx * vz);
    T tz = 2 * (qx * vy - qy * vx);

    // v' = v + q.w * t + cross(q.xyz, t)
    T res_x = vx + qw * tx + (qy * tz - qz * ty);
    T res_y = vy + qw * ty + (qz * tx - qx * tz);
    T res_z = vz + qw * tz + (qx * ty - qy * tx);

    return VecType<FrameA, T>(res_x, res_y, res_z);
}

template <typename FrameA, typename FrameB, typename FrameC, typename T>
Rotation<FrameA, FrameC, T> operator*(const Rotation<FrameA, FrameB, T>& R_ab,
                                      const Rotation<FrameB, FrameC, T>& R_bc) {
    // Delegate the raw mathematical composition to the UnitQuaternion's operator*.
    // The template parameters on the Rotation classes ensure that this operation
    // is only valid if the inner frames (FrameB) match.
    return Rotation<FrameA, FrameC, T>(R_ab.to_quaternion() * R_bc.to_quaternion());
}

}  // namespace refx

#endif /* _REFX_GEOMETRY_INTERNAL_ROTATIONS_ */
