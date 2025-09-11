#ifndef _REFX_TRANSFORMATIONS_INTERNAL_FRAME_CONVERTER_
#define _REFX_TRANSFORMATIONS_INTERNAL_FRAME_CONVERTER_

#include <type_traits>

#include "../../frames/axis.h"
#include "../../frames/internal/frame_validators.h"
#include "../../geometry/coordinate.h"
#include "shuffler_helper.hpp"

namespace refx {
namespace internal {

/**
 * @brief Internal helper for frame-to-frame conversion.
 */
template <typename frameTo, typename frameFrom>
struct FrameConverter {
    using AxisTo = typename frameTo::axis;
    using AxisFrom = typename frameFrom::axis;
    template <template <class, class> class VecType, typename T = double>
    static VecType<frameTo, T> convert(const VecType<frameFrom, T>& from) {
        if constexpr (std::is_same_v<frameTo, frameFrom>) {
            return from;
        }
        FrameDirectionalAxisValidator<typename frameTo::axis, typename frameFrom::axis>::validate();
        // here the magic of axis shuffle happens!
        return VecType<frameTo, T>{// Target X component: Project 'from' onto the 'To' X-direction
                                   get_value_along<AxisTraits<AxisTo>::X>(from),
                                   // Target Y component: Project 'from' onto the 'To' Y-direction
                                   get_value_along<AxisTraits<AxisTo>::Y>(from),
                                   // Target Z component: Project 'from' onto the 'To' Z-direction
                                   get_value_along<AxisTraits<AxisTo>::Z>(from)};
    }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_INTERNAL_FRAME_CONVERTER_ */
