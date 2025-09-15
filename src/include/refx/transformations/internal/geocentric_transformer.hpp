#ifndef _REFX_TRANSFORMATIONS_INTERNAL_GEOCENTRIC_TRANSFORMER_
#define _REFX_TRANSFORMATIONS_INTERNAL_GEOCENTRIC_TRANSFORMER_

#include <cmath>

#include "../../frames/internal/validators.h"
#include "../../geometry/coordinate.h"
#include "../../math/angles.h"
#include "../../models/earth_model/earth_model.h"

namespace refx {
namespace internal {

/**
 * @brief Compile-time geocentric frame transformer.
 *
 * Transforms coordinates between geocentric frames (LLA, LLD, ECEF).
 * Primary template triggers a static_assert if no specialization exists.
 *
 * @tparam frameTo Destination geocentric frame
 * @tparam frameFrom Source geocentric frame
 * @tparam T Numeric type (default: double)
 */
template <typename frameTo, typename frameFrom>
struct GeocentricTransformer {
    template <typename T>
    static Coordinate3D<frameTo, T> transform(const Coordinate3D<frameFrom, T>& from,
                                              const EarthModel<T>& em) {
        if constexpr (std::is_same_v<frameTo, frameFrom>) {
            return from;
        } else {
            static_assert(
                false, "GeocentricTransformer::transform not implemented for the types specified");
        }
    }
};

//
// ---------------------- Geocentric Transformer Implementations ----------------------
//

// LLA <-> LLD conversion (simple axis inversion for "down" vs altitude)
template <>
struct GeocentricTransformer<lld, lla> {
    template <typename T>
    static Coordinate3D<lld, T> transform(const Coordinate3D<lla, T>& lla_,
                                          const EarthModel<T>& em) {
        return Coordinate3D<lld, T>(lla_.latitude(AngleUnit::Deg), lla_.longitude(AngleUnit::Deg),
                                    -lla_.altitude());
    }
};

template <>
struct GeocentricTransformer<lla, lld> {
    template <typename T>
    static Coordinate3D<lla, T> transform(const Coordinate3D<lld, T>& lld_,
                                          const EarthModel<T>& em) {
        return Coordinate3D<lla, T>(lld_.latitude(AngleUnit::Deg), lld_.longitude(AngleUnit::Deg),
                                    -lld_.down());
    }
};

// Compute auxiliary radius for ECEF conversion
template <typename T>
T auxiliary_radius(T lat, const EarthModel<T>& em) {
    const T ecc = em.reference_ellipsoid().eccentricity();
    const T ecc_sqr = ecc * ecc;
    const T slat = std::sin(lat);
    const T slat_sqr = slat * slat;

    const T den = std::sqrt(1.0 - ecc_sqr * slat_sqr);

    return em.equatorial_radius() / den;
}

// LLA -> ECEF conversion
template <>
struct GeocentricTransformer<ecef, lla> {
    template <typename T>
    static Coordinate3D<ecef, T> transform(const Coordinate3D<lla, T>& lla,
                                           const EarthModel<T>& em) {
        const T ecc = em.reference_ellipsoid().eccentricity();
        const T ecc_sqr = ecc * ecc;
        const T lat = lla.latitude(AngleUnit::Rad), lon = lla.longitude(AngleUnit::Rad),
                alt = lla.altitude();
        const T slat = std::sin(lat), clat = std::cos(lat);
        const T slon = std::sin(lon), clon = std::cos(lon);

        const T beta_comp = auxiliary_radius<T>(lat, em);
        const T xe = (beta_comp + alt) * clat * clon;
        const T ye = (beta_comp + alt) * clat * slon;
        const T ze = (beta_comp * (1.0 - ecc_sqr) + alt) * slat;
        return Coordinate3D<ecef, T>(xe, ye, ze);
    }
};

// ECEF -> LLA conversion using Bowring's closed form
template <>
struct GeocentricTransformer<lla, ecef> {
    template <typename T>
    static Coordinate3D<lla, T> transform(const Coordinate3D<ecef, T>& ecef_,
                                          const EarthModel<T>& em) {
        // WGS-84 ellipsoid parameters
        const T a = em.reference_ellipsoid().semi_major_axis();  // semi-major axis [m]
        const T b = em.reference_ellipsoid().semi_minor_axis();  // semi-major axis [m]
        const T e = em.reference_ellipsoid().eccentricity();
        const T e2 = e * e;
        const T ep2 = (a * a - b * b) / (b * b);  // second eccentricity squared
        const T x = ecef_.x();
        const T y = ecef_.y();
        const T z = ecef_.z();

        T lon = std::atan2(y, x);
        T p = std::sqrt(x * x + y * y);

        // Initial parametric latitude (Bowring)
        T theta = std::atan2(z * a, p * b);

        // Geodetic latitude
        T sinTheta = std::sin(theta);
        T cosTheta = std::cos(theta);
        T lat = std::atan2(z + ep2 * b * sinTheta * sinTheta * sinTheta,
                           p - e2 * a * cosTheta * cosTheta * cosTheta);

        // Prime vertical radius of curvature
        T sinLat = std::sin(lat);
        T N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);

        // Altitude
        T alt = p / std::cos(lat) - N;

        return Coordinate3D<lla, T>::from_radians(lat, lon, alt);
    }
};

}  // namespace internal
}  // namespace refx

#endif /* _REFX_TRANSFORMATIONS_INTERNAL_GEOCENTRIC_TRANSFORMER_ */
