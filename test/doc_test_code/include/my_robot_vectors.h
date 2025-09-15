#ifndef _INCLUDE_MY_ROBOT_VECTORS_
#define _INCLUDE_MY_ROBOT_VECTORS_

#include <refx/geometry/vector.h>

#include "my_robot_frames.h"

namespace refx {  // Specialization must be done in the `refex` namespace
/**
 * @brief Specialization of Vector3D for our custom laser_scanner frame.
 * @details Provides semantic accessors matching the underlying axis_flu system.
 */
template <typename T>
struct Vector3D<my_robot::laser_scanner, T> : public internal::VectorContainer3D<T> {
    // Standard boilerplate for a specialization
    using frame = my_robot::laser_scanner;
    using internal::VectorContainer3D<T>::VectorContainer3D;

    // Optional constructor for convenience
    Vector3D(T forward = 0, T left = 0, T up = 0)
        : internal::VectorContainer3D<T>({forward, left, up}) {}

    // --- Semantic Accessors ---
    T& laser_front() { return this->x(); }
    T& laser_side() { return this->y(); }
    T& laser_up() { return this->z(); }

    T laser_front() const { return this->x(); }
    T laser_side() const { return this->y(); }
    T laser_up() const { return this->z(); }
};
}  // namespace refx

#endif /* _INCLUDE_MY_ROBOT_VECTORS_ */
