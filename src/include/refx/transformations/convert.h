#ifndef _REFX_TRANSFORMATIONS_CONVERT_
#define _REFX_TRANSFORMATIONS_CONVERT_

#include "../geometry/coordinate.h"
#include "internal/frame_converter.hpp"
/**
 * @file convert.h
 * @brief Provides the high-level `frame_cast` function for **co-origin** frame conversions.
 * @details This file defines the primary user-facing API for performing simple conversions
 * between different coordinate frames that share the same origin. The `frame_cast`
 * function is designed to be an expressive and type-safe tool for re-expressing a
 * vector or coordinate in a new frame via simple axis shuffling and rotations.
 *
 * @note This function should be strictly used for conversions that do **not** involve a
 * change of origin or a projection (e.g., NED <-> ENU, FRD <-> FLU). For more complex
 * transformations that require a physical Earth model and an origin point (e.g., LLA -> NED),
 * use the `refx::frame_transform` function instead.
 */

namespace refx {

/**
 * @brief Re-expresses a vector or coordinate in a new, co-origin reference frame.
 * @details This function serves as the sole entry point for performing compile-time
 * conversions between frames that share the same origin. It uses a syntax similar to
 * C++'s `static_cast` for familiarity and ease of use.
 *
 * The actual conversion logic is selected at compile-time by dispatching to the
 * appropriate specialization of the internal `internal::FrameConverter` engine. If a
 * conversion between the specified `frameFrom` and `frameTo` has not been defined,
 * this will result in a compile-time error, ensuring that only valid and mathematically
 * correct conversions can be performed.
 *
 * @tparam frameTo The destination frame tag type (e.g., `enu`).
 * @tparam frameFrom The source frame tag type (e.g., `ned`).
 * @tparam VecType The high-level template type (`Vector3D` or `Coordinate3D`).
 * @tparam T The underlying scalar type (e.g., `double`).
 * @param from The object (vector or coordinate) to be converted.
 * @return A new object of `VecType<frameTo, T>` representing the original quantity
 * in the new reference frame.
 */

template <typename frameTo, typename frameFrom, template <class, class> class VecType, typename T>
VecType<frameTo, T> frame_cast(const VecType<frameFrom, T>& from) {
    // if the following pass, then the frameTo and frameFrom are safe to be passed to the function
    internal::FrameTagValidator<frameTo::tag, frameFrom::tag>::validate();
    internal::FrameDirectionalAxisValidator<frameTo, frameFrom>::validate();

    return internal::FrameConverter<frameTo, frameFrom>::convert(from);
}

}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_CONVERT_ */
