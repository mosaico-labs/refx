#ifndef _REFX_MODELS_GRAVITY_MODEL_INTERNAL_GRAVITY_MODEL_
#define _REFX_MODELS_GRAVITY_MODEL_INTERNAL_GRAVITY_MODEL_

#include <cmath>

#include "../gravity_model.h"

namespace refx {

template <typename T>
GravityModel<T>::GravityModel(T gamma_e, T gamma_p, T GM, const ReferenceEllipsoid<T>& datum)
    : m_gamma_e(gamma_e), m_gamma_p(gamma_p), m_GM(GM), m_datum(datum) {
    const T num = m_datum.semi_minor_axis() * m_gamma_p - m_datum.semi_major_axis() * m_gamma_e;
    const T den = m_datum.semi_major_axis() * m_gamma_e;
    m_somigliana_k = num / den;
}

template <typename T>
T GravityModel<T>::normal_gravity_on_ellipsoid(T lat) const {
    const T ecc = m_datum.eccentricity();
    const T ecc_sqr = ecc * ecc;
    const T slat = std::sin(lat);
    const T slat_sqr = slat * slat;

    const T denom = std::sqrt(1.0 - ecc_sqr * slat_sqr);
    return m_gamma_e * (1.0 + m_somigliana_k * slat_sqr) / denom;
}

template <typename T>
T GravityModel<T>::apply_height(T gamma_phi, T lat, T h, GravityHeightCorrection hc) const {
    if (hc == GravityHeightCorrection::None) {
        return gamma_phi;
    }

    if (hc == GravityHeightCorrection::FreeAir) {
        const T x = h / m_datum.semi_major_axis();
        return gamma_phi * (1.0 - 2.0 * x + 3.0 * x * x);
    }
    if (hc == GravityHeightCorrection::Ellipsoidal) {
        const T f = 1.0 / m_datum.inverse_flattening();
        const T omega = m_datum.angular_velocity();
        const T a = m_datum.semi_major_axis();
        const T b = m_datum.semi_minor_axis();
        const T m = (omega * omega * a * a * b) / m_GM;
        const T slat = std::sin(lat);

        const T term = (1.0 + f + m - 2.0 * f * slat * slat);
        return gamma_phi * (1.0 - (2.0 / a) * term * h + (3.0 / (a * a)) * h * h);
    }

    return gamma_phi;
}

}  // namespace refx

#endif /* _REFX_MODELS_GRAVITY_MODEL_INTERNAL_GRAVITY_MODEL_ */
