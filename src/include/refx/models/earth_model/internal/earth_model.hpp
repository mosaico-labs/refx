#ifndef _REFX_MODELS_EARTH_MODEL_INTERNAL_EARTH_MODEL_
#define _REFX_MODELS_EARTH_MODEL_INTERNAL_EARTH_MODEL_

#include <cmath>

#include "../earth_model.h"

namespace refx {

template <typename T>
const ReferenceEllipsoid<T>& EarthModel<T>::reference_ellipsoid() const {
    return m_datum;
}

template <typename T>
const GravityModel<T>& EarthModel<T>::gravity_model() const {
    return m_gravity_model;
}

template <typename T>
T EarthModel<T>::equatorial_radius() const {
    return m_datum.semi_major_axis();
}

template <typename T>
T EarthModel<T>::polar_radius() const {
    return m_datum.semi_minor_axis();
}

template <typename T>
T EarthModel<T>::mean_radius() const {
    const T num = 2.0 * m_datum.semi_major_axis() + m_datum.semi_minor_axis();
    return num / 3.0;
}

template <typename T>
T EarthModel<T>::meridian_radius(T latitude) const {
    const T a = m_datum.semi_major_axis();
    const T ecc = m_datum.eccentricity();
    const T slat = std::sin(latitude);
    const T ecc_sqr = ecc * ecc;
    const T sin_sqr = slat * slat;
    const T num = a * (1.0 - ecc_sqr);
    const T den_inner = 1.0 - ecc_sqr * sin_sqr;
    // compute den^(3/2) as sqrt(den) * den
    const T den = std::sqrt(den_inner) * den_inner;

    return num / den;
}

template <typename T>
T EarthModel<T>::normal_radius(T latitude) const {
    const T ecc = m_datum.eccentricity();
    const T ecc_sqr = ecc * ecc;
    const T slat = std::sin(latitude);
    const T sin_sqr = slat * slat;
    const T den = sqrt(1.0 - ecc_sqr * sin_sqr);
    const T num = equatorial_radius();

    return num / den;
}

template <typename T>
T EarthModel<T>::gravity(const T lat) const {
    return m_gravity_model.normal_gravity_on_ellipsoid(lat);
}

template <typename T>
T EarthModel<T>::gravity(const Coordinate3D<lla, T>& latlonalt,
                         const GravityHeightCorrection& hc) const {
    const T lat = latlonalt.latitude(AngleUnit::Rad);
    const T grav_0 = m_gravity_model.normal_gravity_on_ellipsoid(lat);
    return m_gravity_model.apply_height(grav_0, lat, latlonalt.altitude(), hc);
}

}  // namespace refx

#endif /* _REFX_MODELS_EARTH_MODEL_INTERNAL_EARTH_MODEL_ */
