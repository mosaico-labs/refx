#ifndef _REFX_TRANSFORMATIONS_INTERNAL_FRAME_TRANSFORMER_
#define _REFX_TRANSFORMATIONS_INTERNAL_FRAME_TRANSFORMER_

#include <cmath>

#include "../../frames/internal/frame_validators.h"
#include "../../geometry/coordinate.h"
#include "../../math/angles.h"
#include "../../models/earth_model/earth_model.h"
#include "canonical_frames.h"
#include "frame_converter.hpp"

namespace refx {
namespace internal {

/**
 * @brief Compile-time co-located frame transformer.
 *
 * Transforms coordinates between Cartesian and aer-like frames.
 * Primary template triggers a static_assert if no specialization exists.
 *
 * @tparam frameTo Destination geocentric frame
 * @tparam frameFrom Source geocentric frame
 * @tparam T Numeric type (default: double)
 */
template <typename axisTo, typename axisFrom, typename frameTo, typename frameFrom>
struct FrameTransformer {
    template <template <class, class> class VecType, typename T>
    static VecType<frameTo, T> transform(const VecType<frameFrom, T>& from) {
        static_assert(
            false,
            "FrameTransformer::transform not implemented for the types specified. "
            "Parameters must have `axis_aer` and any cartesian (non geocentric) semantics.");
    }
};

/**
 * @brief Convenience alias for transforming into the canonical frame.
 *
 * Selects the specialization of `FrameTransformer` that maps from an
 * `axis_aer` frame into the canonical Cartesian `axis_frd` frame
 * associated with the given `TagTo`.
 *
 * @tparam TagTo Destination frame tag (resolved via `canonical_frame<TagTo>::frame`)
 * @tparam frameFrom Source frame type
 */
template <FrameTag TagTo, typename frameFrom>
using FrameTransformerToCanonical =
    FrameTransformer<axis_frd, axis_aer, typename canonical_frame<TagTo>::frame, frameFrom>;

/**
 * @brief Convenience alias for transforming out of the canonical frame.
 *
 * Selects the specialization of `FrameTransformer` that maps from the
 * canonical Cartesian `axis_frd` frame associated with `TagFrom` into
 * a target frame with `axis_aer` semantics.
 *
 * @tparam frameTo Destination frame type
 * @tparam TagFrom Source frame tag (resolved via `canonical_frame<TagFrom>::frame`)
 */
template <typename frameTo, FrameTag TagFrom>
using FrameTransformerFromCanonical =
    FrameTransformer<axis_aer, axis_frd, frameTo, typename canonical_frame<TagFrom>::frame>;

/// @brief specialization for the canonical conversion from `axis_aer` to `axis_frd`
template <typename frameTo, typename frameFrom>
struct FrameTransformer<axis_frd, axis_aer, frameTo, frameFrom> {
    template <template <class, class> class VecType, typename T>
    static VecType<frameTo, T> transform(const VecType<frameFrom, T>& from) {
        // validate the out desired frame to be of the right type
        FrameAxisValidator<typename frameTo::axis, axis_frd>::validate();

        const T az = deg2rad(from.x());
        const T el = deg2rad(from.y());
        const T rg = from.z();
        const T r_cos_el = rg * std::cos(el);
        const T rgt = r_cos_el * std::sin(az);
        const T fwd = r_cos_el * std::cos(az);
        const T dwn = -rg * std::sin(el);
        return VecType<frameTo, T>{fwd, rgt, dwn};
    }
};

/// @brief specialization for the canonical conversion from `axis_frd` to `axis_aer`
template <typename frameTo, typename frameFrom>
struct FrameTransformer<axis_aer, axis_frd, frameTo, frameFrom> {
    template <template <class, class> class VecType, typename T>
    static VecType<frameTo, T> transform(const VecType<frameFrom, T>& from) {
        // validate the input frame to be of the right type
        FrameAxisValidator<typename frameFrom::axis, axis_frd>::validate();

        const T rgt = from.y(), fwd = from.x(), up = -from.z();
        const T r = std::hypot(rgt, fwd);
        const T rg = std::hypot(r, up);
        const T el = std::atan2(up, r);
        const T two_pi = 2.0 * M_PI;
        const T az = std::fmod(two_pi + std::fmod(std::atan2(rgt, fwd), two_pi), two_pi);
        return VecType<frameTo, T>{rad2deg(az), rad2deg(el), rg};
    }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_INTERNAL_FRAME_TRANSFORMER_ */
