#ifndef _REFX_GEOMETRY_INTERNAL_EULER_BASE_
#define _REFX_GEOMETRY_INTERNAL_EULER_BASE_

#include "vector_base.h"

/**
 * @file euler_base.h
 * @brief Defines the internal, non-user-facing base class for Euler angles storage.
 * @details This internal header provides the `EulerBase` class, which serves as a
 * common storage and interface layer for the various `EulerAngles` specializations.
 * Its purpose is to provide generic, non-semantic accessors (`.angle_x()`, etc.)
 * to the underlying data, which is stored in a `VectorContainer3D`.
 *
 * @warning This class is an implementation detail of the `EulerAngles` struct and is
 * not intended for direct use by end-users.
 */

namespace refx {
namespace internal {

/**
 * @brief A low-level base class providing storage for three angular values.
 * @tparam T The scalar type of the angles (e.g., `float`, `double`).
 *
 * @details This class inherits from `VectorContainer3D<T>` to reuse its data
 * storage. It provides a common interface with generic `angle_x`, `angle_y`, and
 * `angle_z` accessors. The high-level `EulerAngles` specializations inherit from
 * this class and map these generic accessors to semantic ones (e.g., `roll`, `pitch`, `yaw`)
 * according to their specific rotation sequence.
 */
template <typename T = double>
struct EulerBase : public VectorContainer3D<T> {
    // Inherit all constructors from the 3D vector base.
    using VectorContainer3D<T>::VectorContainer3D;
    using container_type = typename internal::VectorContainer3D<T>::container_type;
    // --- Generic Named Accessors ---

    /// @brief Provides mutable access to the angle for rotation about the X-axis.
    T& angle_x() { return this->x(); }
    /// @brief Provides mutable access to the angle for rotation about the Y-axis.
    T& angle_y() { return this->y(); }
    /// @brief Provides mutable access to the angle for rotation about the Z-axis.
    T& angle_z() { return this->z(); }

    /// @brief Provides const access to the angle for rotation about the X-axis.
    const T& angle_x() const { return this->x(); }
    /// @brief Provides const access to the angle for rotation about the Y-axis.
    const T& angle_y() const { return this->y(); }
    /// @brief Provides const access to the angle for rotation about the Z-axis.
    const T& angle_z() const { return this->z(); }

    /**
     * @brief Provides const access to the underlying data container.
     * @return A const reference to the internal `std::array` storing angle components.
     */
    const container_type& data() const { return internal::VectorContainer3D<T>::data; }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_GEOMETRY_INTERNAL_EULER_BASE_ */
