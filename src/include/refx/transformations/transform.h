#ifndef _REFX_TRANSFORMATIONS_TRANSFORM_
#define _REFX_TRANSFORMATIONS_TRANSFORM_

#include "../frames/internal/traits.h"  // For is_semantic_axis_v, is_directional_axis_v
#include "../geometry/transformation.h"
#include "../models/earth_model/earth_model.h"
#include "internal/frame_transformer.hpp"
#include "internal/geocentric_transformer.hpp"
#include "internal/local_tangent_transformer.hpp"

/**
 * @file transform.h
 * @brief Provides the high-level `frame_transform` function for complex coordinate transformations.
 * @details This file defines the primary user-facing API for performing complex
 * coordinate transformations that involve a **change of origin** and require a
 * physical `EarthModel` as context.
 *
 * This is fundamentally different from `frame_cast`, which only handles simple,
 * co-origin rotations. `frame_transform` is used for essential geodetic operations
 * such as converting a global LLA coordinate to a local NED coordinate relative to a
 * specific tangent point.
 */

namespace refx {

/**
 * @brief Transforms a vector/coordinate between two co-located frames (same origin) with different
 * axis semantics (axis_aer <-> cartesian). Must be used when converting from
 * Azimuth-Eleveation-Range (AER)-like frames to Cartesian frames (NED, FRD, ...).
 *
 * @tparam frameTo The destination frame (e.g., `aer`).
 * @tparam frameFrom The source frame (e.g., `ned`).
 * @tparam T The underlying scalar type.
 * @param from The coordinate to be transformed.
 * @return A new `VecType<frameTo>` representing the transformed entity.
 *
 * @note Cannot be used to convert geocentric coordinates (which need an EarthModel)
 * @see frame_transform(const Coordinate3D<frameFrom, T>&, const EarthModel<T>&);
 */
template <typename frameTo, typename frameFrom, template <class, class> class VecType, typename T>
VecType<frameTo, T> frame_transform(const VecType<frameFrom, T>& from) {
    // evaluate the same frame tag
    internal::FrameTagValidator<frameTo::tag, frameFrom::tag>::validate();
    // evaluate just one must have semantic axis
    internal::FrameSemanticAxisValidator<frameTo, frameFrom,
                                         internal::ValidateCondition::XOR>::validate();
    // extract the canonical frame for the tag
    using frameCanonical = typename internal::canonical_frame<frameTo::tag>::frame;
    // convert to cartesian
    if constexpr (is_semantic_axis_v<typename frameFrom::axis>) {
        return internal::FrameConverter<frameTo, frameCanonical>::convert(
            internal::FrameTransformerToCanonical<frameTo::tag, frameFrom>::transform(from));
    } else {  // non semantic: already validated
        // convert to aer
        return internal::FrameTransformerFromCanonical<frameTo, frameFrom::tag>::transform(
            internal::FrameConverter<frameCanonical, frameFrom>::convert(from));
    }
}

/**
 * @brief Transforms a coordinate between two **geocentric** frames (e.g., ECEF ↔ LLA).
 * @details This function handles transformations between frames that share the Earth's
 * center as their origin. It relies on the provided `EarthModel` to supply the correct
 * reference ellipsoid parameters for the conversion.
 *
 * @tparam frameTo The destination geocentric frame tag type (e.g., `ecef`).
 * @tparam frameFrom The source geocentric frame tag type (e.g., `lla`).
 * @tparam T The underlying scalar type.
 * @param from The coordinate to be transformed.
 * @param em The `EarthModel` (e.g., `EarthModelWGS84`) providing the geodetic context.
 * @return A new `Coordinate3D<frameTo>` representing the transformed position.
 */
template <typename frameTo, typename frameFrom, typename T>
Coordinate3D<frameTo, T> frame_transform(const Coordinate3D<frameFrom, T>& from,
                                         const EarthModel<T>& em) {
    // validate both geocentric
    internal::FrameTagValidator<frameTo::tag, frameFrom::tag, FrameTag::Geocentric>::validate();
    return internal::GeocentricTransformer<frameTo, lla>::transform(
        internal::GeocentricTransformer<lla, frameFrom>::transform(from, em), em);
}

/**
 * @brief Transforms a coordinate between a geocentric frame and a local tangent frame.
 * @details Projects a global coordinate (like LLA or ECEF) onto a local Cartesian plane
 * (like NED or ENU) centered at a specific `origin`, or performs the inverse operation.
 *
 * @tparam frameTo The destination frame tag type (can be local or geocentric).
 * @tparam frameFrom The source frame tag type (can be local or geocentric).
 * @tparam frameOrigin The frame of the provided `origin` coordinate.
 * @tparam T The underlying scalar type.
 * @param from The coordinate to be transformed.
 * @param origin The geodetic coordinate that defines the origin (0,0,0) of the local
 * tangent frame.
 * @param em The `EarthModel` providing the geodetic context.
 * @return A new `Coordinate3D<frameTo>` representing the transformed position.
 */
template <typename frameTo, typename frameFrom, typename frameOrigin, typename T>
Coordinate3D<frameTo, T> frame_transform(const Coordinate3D<frameFrom, T>& from,
                                         const Coordinate3D<frameOrigin, T>& origin,
                                         const EarthModel<T>& em) {
    using frameCanonical = typename internal::canonical_frame<FrameTag::LocalTangent>::frame;

    // at least one between frameTo and frameFrom must be of different tag
    internal::FrameTagValidator<frameOrigin::tag, FrameTag::Geocentric>::validate();
    internal::FrameTagValidator<frameTo::tag, frameFrom::tag, FrameTag::Geocentric,
                                internal::ValidateCondition::XOR>::validate();
    internal::FrameTagValidator<frameTo::tag, frameFrom::tag, FrameTag::LocalTangent,
                                internal::ValidateCondition::XOR>::validate();
    // convert to local-tangent
    if constexpr (frameFrom::tag == FrameTag::Geocentric) {
        return internal::FrameConverter<
            frameTo, frameCanonical>::convert(  // convert from canonical to frameTo (local)
            internal::LocalTangentTransformerToCanonical<
                frameOrigin>::transform(  // convert to canonical (local) from lla
                internal::GeocentricTransformer<lla, frameFrom>::transform(
                    from, em),  // convert to lla (local) from frameFrom
                origin, em));
        // convert to geocentric
    } else if constexpr (frameFrom::tag == FrameTag::LocalTangent) {
        return internal::GeocentricTransformer<frameTo, lla>::transform(
            internal::LocalTangentTransformerFromCanonical<frameOrigin>::transform(
                internal::FrameConverter<frameCanonical, frameFrom>::convert(from), origin, em),
            em);
    } else {
        static_assert(
            false,
            "Incompatible frameFrom tag, must be FrameTag::Geocentric OR FrameTag::LocalTangent");
    }
}

/**
 * @brief Transforms a vector/coordinate between two Cartesian frames using a full SE(3) pose.
 * @details This function serves as operator for transforming physical quantities
 * between two Cartesian frames (e.g., from a vehicle's body to the world).
 * It uses a `Transformation` object to provide the complete pose (the SE(3)
 * context) that defines the relationship between the source and destination frames.
 * @tparam frameTo The destination (target) Cartesian frame tag type.
 * @tparam frameFrom The source Cartesian frame tag type.
 * @tparam VecType The underlying vector type (Vector3D/Coordinate3D).
 * @tparam T The underlying scalar type for calculations (e.g., double, float).
 * @param from The `Vector3D/Coordinate3D` to be transformed.
 * @param pose The `Transformation<frameTo, frameFrom, T>` object that defines
 * the complete pose (orientation and position) of the `frameFrom`
 * relative to the `frameTo`.
 * @return A new `Vector3D/Coordinate3D<frameTo>` representing the `from` vector expressed
 * in the destination frame.
 */
template <typename frameTo, typename frameFrom, template <class, class> class VecType, typename T>
VecType<frameTo, T> frame_transform(const Transformation<frameTo, frameFrom, T>& pose,
                                    const VecType<frameFrom, T>& p_from) {
    return pose * p_from;
}
}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_TRANSFORM_ */
