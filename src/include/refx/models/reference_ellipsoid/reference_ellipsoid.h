#ifndef _REFX_MODELS_REFERENCE_ELLIPSOID_REFERENCE_ELLIPSOID_
#define _REFX_MODELS_REFERENCE_ELLIPSOID_REFERENCE_ELLIPSOID_

#include <string>

/**
 * @file reference_ellipsoid.h
 * @brief Defines the ReferenceEllipsoid class for modeling the Earth's geometry.
 * @details This file provides the `refx::ReferenceEllipsoid` class, which stores the
 * defining geometric and physical constants of an Earth-approximating ellipsoid.
 * This model of the Earth as an oblate spheroid is the foundational datum for all
 * high-accuracy geodetic computations, including coordinate transformations (e.g.,
 * LLA to ECEF) and gravity calculations.
 */

namespace refx {

/**
 * @brief A mathematical model of the Earth's shape as an oblate spheroid. ジオイド
 * @tparam T The scalar type for all constants (e.g., `float`, `double`).
 *
 * @details This class encapsulates the set of parameters that define a specific
 * reference ellipsoid. An ellipsoid is a simplified mathematical surface that
 * approximates the Earth's true shape (the geoid). It serves as the fundamental
 * geometric reference for a geodetic datum.
 *
 * All transformations between geodetic coordinates (like LLA) and Cartesian frames
 * (like ECEF) are dependent on the parameters of a specific ellipsoid. This class
 * provides a clean, type-safe way to manage and access these critical constants.
 */
template <typename T = double>
struct ReferenceEllipsoid {
    /**
     * @brief Constructs a reference ellipsoid from its defining physical constants.
     * @param name The official name of the ellipsoid standard (e.g., "WGS-84").
     * @param semi_major_axis The equatorial radius, denoted 'a', in [m].
     * @param semi_minor_axis The polar radius, denoted 'b', in [m].
     * @param inverse_flattening The inverse of the flattening factor, `1/f`.
     * @param eccentricity The first eccentricity of the ellipsoid, `e`.
     * @param angular_velocity The mean angular velocity of the Earth, `ω`, in [rad/s].
     * @param reference_epoch The official epoch for which the constants are defined.
     */
    ReferenceEllipsoid(std::string name, T semi_major_axis, T semi_minor_axis, T inverse_flattening,
                       T eccentricity, T angular_velocity, std::string reference_epoch)
        : m_semi_major_axis(semi_major_axis),
          m_semi_minor_axis(semi_minor_axis),
          m_inverse_flattening(inverse_flattening),
          m_eccentricity(eccentricity),
          m_angular_velocity(angular_velocity),
          m_reference_epoch(reference_epoch),
          m_name(name) {}

    /**
     * @brief The default constructor is deleted to ensure that an ellipsoid is
     * always explicitly initialized with a complete and valid set of constants.
     */
    ReferenceEllipsoid() = delete;

    // --- Accessors for Defining Constants ---

    /// @brief Returns the semi-major axis (equatorial radius 'a') in [m].
    T semi_major_axis() const { return m_semi_major_axis; }

    /// @brief Returns the semi-minor axis (polar radius 'b') in [m].
    T semi_minor_axis() const { return m_semi_minor_axis; }

    /// @brief Returns the inverse flattening `1/f`, a measure of the ellipsoid's compression.
    T inverse_flattening() const { return m_inverse_flattening; }

    /// @brief Returns the first eccentricity `e`, defined as `sqrt(a²-b²)/a`.
    T eccentricity() const { return m_eccentricity; }

    /// @brief Returns the mean angular velocity of the Earth `ω` in [rad/s].
    T angular_velocity() const { return m_angular_velocity; }

    /// @brief Returns the reference epoch (e.g., "2024.00") for the constant definitions.
    const std::string& reference_epoch() const { return m_reference_epoch; }

    /// @brief Returns the official name of the reference ellipsoid (e.g., "WGS-84").
    const std::string& name() const { return m_name; }

   private:
    T m_semi_major_axis;
    T m_semi_minor_axis;
    T m_inverse_flattening;
    T m_eccentricity;
    T m_angular_velocity;
    std::string m_reference_epoch;
    std::string m_name;
};

/*==========================================================================*/
/*============== Pre-defined Reference Ellipsoid Standards =================*/
/*==========================================================================*/

/**
 * @brief The **World Geodetic System 1984 (WGS-84)** reference ellipsoid.
 * @details This is the standard reference ellipsoid for the Global Positioning System
 * (GPS) and is the most widely used datum in modern navigation, robotics, and GIS.
 * Unless an application specifically requires a different datum, this should be the
 * default choice.
 */
template <typename T = double>
struct ReferenceEllipsoidWGS84 : public ReferenceEllipsoid<T> {
    /// @brief Constructs a WGS-84 ellipsoid with its standard defining constants.
    ReferenceEllipsoidWGS84()
        : ReferenceEllipsoid<T>("WGS-84", 6378137.0, 6356752.314245, 298.257223563, 0.0818191908426,
                                7.292115e-5, "2024.00") {}
};

/**
 * @brief The **Geodetic Reference System 1980 (GRS-80)** reference ellipsoid.
 * @details A global reference ellipsoid adopted by the International Union of Geodesy
 * and Geophysics. It is very similar to WGS-84, with minor differences in some
 * defining constants. It is widely used in geodesy and serves as the basis for
 * datums like NAD-83.
 */
template <typename T = double>
struct ReferenceEllipsoidGRS80 : public ReferenceEllipsoid<T> {
    /// @brief Constructs a GRS-80 ellipsoid with its standard defining constants.
    ReferenceEllipsoidGRS80()
        : ReferenceEllipsoid<T>("GRS-80", 6378137.0, 6356752.314140, 298.257222101,
                                0.0818191908426215, 7.292115e-5, "1989.00") {}
};

}  // namespace refx

#endif /* _REFX_MODELS_REFERENCE_ELLIPSOID_REFERENCE_ELLIPSOID_ */
