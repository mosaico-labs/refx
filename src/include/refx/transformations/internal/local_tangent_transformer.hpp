#ifndef _REFX_TRANSFORMATIONS_INTERNAL_LOCAL_TANGENT_TRANSFORMER_
#define _REFX_TRANSFORMATIONS_INTERNAL_LOCAL_TANGENT_TRANSFORMER_

#include <cmath>

#include "../../frames/internal/validators.h"
#include "../../geometry/coordinate.h"
#include "../../math/angles.h"
#include "../../models/earth_model/earth_model.h"
#include "canonical_frames.h"
#include "frame_converter.hpp"

namespace refx {
namespace internal {

/**
 * @brief Compile-time local tangent plane transformer.
 *
 * Transforms coordinates between geocentric frames (ECEF/LLA) and local tangent planes (NED,
 * ENU, NWU), or vice versa, assuming a given local origin. Implements planar projection and
 * inverse projection.
 *
 * @tparam frameTo Destination frame
 * @tparam frameFrom Source frame
 * @tparam frameOrigin Frame of the origin for local tangent plane
 * @tparam T Numeric type
 */
template <typename frameTo, typename frameFrom, typename frameOrigin>
struct LocalTangentTransformer {
    template <typename T>
    static Coordinate3D<frameTo, T> transform(const Coordinate3D<frameFrom, T>& from,
                                              const Coordinate3D<frameOrigin, T>& origin,
                                              const EarthModel<T>& em) {
        static_assert(false,
                      "LocalTangentTransformer::transform not implemented for the types specified");
    }
};

//
// ---------------------- Local Tangent Transformer Implementations ----------------------
//

/// @brief lla -> local-tangent canonical frame
/// @tparam frameToLocal
/// @tparam frameOrigin
template <typename frameOrigin>
struct LocalTangentTransformer<typename canonical_frame<FrameTag::LocalTangent>::frame, lla,
                               frameOrigin> {
    using frameToLocal = typename canonical_frame<FrameTag::LocalTangent>::frame;
    template <typename T>
    static Coordinate3D<frameToLocal, T> transform(const Coordinate3D<lla, T>& from,
                                                   const Coordinate3D<frameOrigin, T>& origin,
                                                   const EarthModel<T>& em) {
        // validate inputs
        FrameTagValidator<frameToLocal::tag, FrameTag::LocalTangent>::validate();
        FrameTagValidator<frameOrigin::tag, FrameTag::Geocentric>::validate();
        // verify that the canonical frame is what we expect (ned)
        AxisValidator<typename frameToLocal::axis, axis_frd>::validate();

        // transform to LLA regardless of the specific type of `frameOrigin`
        const Coordinate3D<lla, T> orig_lla =
            GeocentricTransformer<lla, frameOrigin>::transform(origin, em);
        const T lat_0 = orig_lla.latitude(AngleUnit::Rad), alt_0 = orig_lla.altitude();
        const auto& delta_pos = from - orig_lla;
        const T north =
            delta_pos.delta_latitude(AngleUnit::Rad) * (em.meridian_radius(lat_0) + alt_0);
        const T east = delta_pos.delta_longitude(AngleUnit::Rad) *
                       (em.normal_radius(lat_0) + alt_0) * std::cos(lat_0);
        const T down = -delta_pos.delta_altitude();
        // transform to the actual `ToLocal` type and return
        return Coordinate3D<frameToLocal, T>(north, east, down);
    }
};
template <typename frameOrigin>
using LocalTangentTransformerToCanonical =
    LocalTangentTransformer<typename canonical_frame<FrameTag::LocalTangent>::frame, lla,
                            frameOrigin>;

/// @brief any local-tangent -> lla
/// @tparam frameFromLocal
/// @tparam frameOrigin
template <typename frameOrigin>
struct LocalTangentTransformer<lla, typename canonical_frame<FrameTag::LocalTangent>::frame,
                               frameOrigin> {
    using frameFromLocal = typename canonical_frame<FrameTag::LocalTangent>::frame;
    template <typename T>
    static Coordinate3D<lla, T> transform(const Coordinate3D<frameFromLocal, T>& from,
                                          const Coordinate3D<frameOrigin, T>& origin,
                                          const EarthModel<T>& em) {
        // validate inputs
        FrameTagValidator<frameFromLocal::tag, FrameTag::LocalTangent>::validate();
        FrameTagValidator<frameOrigin::tag, FrameTag::Geocentric>::validate();
        // verify that the canonical frame is what we expect (ned)
        AxisValidator<typename frameFromLocal::axis, axis_frd>::validate();

        // transform to LLA the origin, regardless of the specific type of `frameOrigin`
        const Coordinate3D<lla, T> orig_lla =
            GeocentricTransformer<lla, frameOrigin>::transform(origin, em);
        // do computations
        const T lat_0 = orig_lla.latitude(AngleUnit::Rad), alt_0 = orig_lla.altitude();
        // compute in LLA, then transform to the actual `frameOrigin` type
        const T den_lat = em.meridian_radius(lat_0) + alt_0;
        const T delta_lat = static_cast<T>(from.north()) / den_lat;
        const T den_lon = (em.normal_radius(lat_0) + alt_0) * std::cos(lat_0);
        const T delta_lon = static_cast<T>(from.east()) / den_lon;

        // obtain the final LLA pos, by composing the deltas with the origin
        const T lat = lat_0 + delta_lat;
        const T lon = orig_lla.longitude(AngleUnit::Rad) + delta_lon;
        const T alt = alt_0 - from.down();  // down to up
        return Coordinate3D<lla, T>::from_radians(lat, lon, alt);
    }
};
template <typename frameOrigin>
using LocalTangentTransformerFromCanonical =
    LocalTangentTransformer<lla, typename canonical_frame<FrameTag::LocalTangent>::frame,
                            frameOrigin>;

}  // namespace internal
}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_INTERNAL_LOCAL_TANGENT_TRANSFORMER_ */
