#ifndef _REFX_TRANSFORMATIONS_INTERNAL_SHUFFLER_HELPER_
#define _REFX_TRANSFORMATIONS_INTERNAL_SHUFFLER_HELPER_

#include "../../frames/axis.h"
#include "../../frames/internal/traits.h"

namespace refx {
namespace internal {

/**
 * @brief Get the value of th input vector/coordinate along one direction.
 * @details This function determines the value of a source vector along a specific
 * target direction. It inspects the source axis type (`AxisFrom`) to find which
 * component (x, y, or z) aligns with the `TargetDir` and with what sign.
 * If no component aligns (due to orthogonality), it generate a compile-time error.
 *
 * @tparam TargetDir The geometric direction we want to project onto.
 * @tparam AxisFrom The axis system of the source vector/coordinate.
 * @param v The source vector/coordinate.
 * @return The scalar value of the source vector/coordinate projected onto the target direction.
 */
template <AxisDirection TargetDir, typename Frame, template <class, class> class VecType,
          typename T>
constexpr T get_value_along(const VecType<Frame, T>& v) {
    using AxisFrom = typename Frame::axis;
    // Check alignment with the source X-axis
    if constexpr (AxisTraits<AxisFrom>::X == TargetDir) {
        return +v.x();
    } else if constexpr (AxisTraits<AxisFrom>::X == opposite<TargetDir>()) {
        return -v.x();
    } else if constexpr (AxisTraits<AxisFrom>::Y == TargetDir) {
        // Check alignment with the source Y-axis
        return +v.y();
    } else if constexpr (AxisTraits<AxisFrom>::Y == opposite<TargetDir>()) {
        return -v.y();
    } else if constexpr (AxisTraits<AxisFrom>::Z == TargetDir) {
        // Check alignment with the source Z-axis
        return +v.z();
    } else if constexpr (AxisTraits<AxisFrom>::Z == opposite<TargetDir>()) {
        return -v.z();
    } else {
        static_assert(false,
                      "Unable to find target direction in input vector/coordinate. Maybe the used "
                      "frame contains incorrect axis direction (non right-handed).");
    }
}

}  // namespace internal

}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_INTERNAL_SHUFFLER_HELPER_ */
