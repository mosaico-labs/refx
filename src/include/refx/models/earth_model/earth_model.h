#ifndef _REFX_MODELS_EARTH_MODEL_EARTH_MODEL_
#define _REFX_MODELS_EARTH_MODEL_EARTH_MODEL_

#include <cmath>

#include "../../geometry/coordinate.h"
#include "../gravity_model/gravity_model.h"
#include "../magnetic_model/magnetic_model.h"
#include "../reference_ellipsoid/reference_ellipsoid.h"

/**
 * @file earth_model.h
 * @brief Defines the central EarthModel class, which consolidates geodetic and physical models.
 * @details This file provides the `refx::EarthModel` class, a high-level utility that
 * serves as the primary context for all geodetic and navigation-related calculations.
 * It encapsulates a specific reference ellipsoid (e.g., WGS-84), a gravity model, and
 * an optional magnetic field model into a single, coherent object.
 */

namespace refx {

/**
 * @brief A unified model of the Earth for high-fidelity navigation computations.
 * @tparam T The scalar type for all calculations (e.g., `float`, `double`).
 *
 * @details This class acts as a central "context" object that combines the three
 * fundamental physical models required for advanced navigation:
 * 1.  **`ReferenceEllipsoid`**: Defines the shape and size of the Earth (e.g., WGS-84).
 * 2.  **`GravityModel`**: Defines the Earth's gravitational field.
 * 3.  **`MagneticFieldModel`**: Defines the Earth's magnetic field.
 *
 * It provides convenient helper functions to compute critical values derived from
 * these models, such as the local gravity vector or the Earth's radii of curvature
 * at a given latitude. This class is an essential input for algorithms like INS
 * mechanization, ECEF-to-NED transformations, and gravity compensation for IMUs.
 */
template <typename T = double>
struct EarthModel {
    /**
     * @brief Constructs an Earth model from its constituent physical models.
     * @param gd The reference ellipsoid defining the Earth's shape (e.g., WGS-84).
     * @param gm The gravity model defining the Earth's gravitational field.
     * @param mm (Optional) The magnetic field model. If not provided, a default,
     * empty model is used.
     */
    EarthModel(const ReferenceEllipsoid<T>& gd, const GravityModel<T>& gm,
               const MagneticFieldModel<T>& mm = MagneticFieldModel<T>())
        : m_datum(gd), m_gravity_model(gm), m_magnetic_model(mm) {}

    /**
     * @brief The default constructor is deleted to ensure that an Earth model is
     * always explicitly initialized with valid physical models.
     */
    EarthModel() = delete;

    // --- Component Model Accessors ---

    /// @brief Returns the reference ellipsoid associated with this Earth model.
    const ReferenceEllipsoid<T>& reference_ellipsoid() const;

    /// @brief Returns the gravity model associated with this Earth model.
    const GravityModel<T>& gravity_model() const;

    // --- Derived Gravity Calculations ---

    /**
     * @brief Computes the magnitude of normal gravity on the surface of the ellipsoid.
     * @details This function uses a closed-form solution (like the Somigliana formula)
     * provided by the underlying `GravityModel` to calculate the theoretical gravity
     * at a specific latitude, assuming zero altitude.
     * @param lat The geodetic latitude in **radians**.
     * @return The magnitude of the local gravity vector in [m/s²].
     */
    T gravity(const T lat) const;

    /**
     * @brief Computes the magnitude of normal gravity at a specific 3D geodetic coordinate.
     * @details This is essential for gravity compensation in an INS, where the gravity
     * vector changes with both latitude and altitude.
     * @param coord The `Coordinate3D<lla>` at which to calculate gravity.
     * @param hc The type of height correction to apply (e.g., Free-Air). If
     * GravityHeightCorrection::None then the gravity at equator is returned,
     * same as gravity(const T lat).
     * @return The magnitude of the local gravity vector in [m/s²].
     */
    T gravity(const Coordinate3D<lla, T>& coord,
              const GravityHeightCorrection& hc = GravityHeightCorrection::None) const;

    // --- Derived Earth Radii Calculations ---

    /// @brief Returns the equatorial radius (semi-major axis) of the ellipsoid in [m].
    T equatorial_radius() const;

    /// @brief Returns the polar radius (semi-minor axis) of the ellipsoid in [m].
    T polar_radius() const;

    /// @brief Returns the mean radius of the ellipsoid `(2a + b) / 3`, in [m].
    T mean_radius() const;

    /**
     * @brief Computes the meridian radius of curvature at a given latitude.
     * @details This is the radius of the curve formed by slicing the ellipsoid along a
     * meridian (a line of constant longitude). It is essential for calculating how
     * North-South velocity translates into a change in latitude.
     * @param latitude The geodetic latitude in **radians**.
     * @return The meridian radius of curvature in [m].
     */
    T meridian_radius(T latitude) const;

    /**
     * @brief Computes the transverse (or prime vertical) radius of curvature.
     * @details This is the radius of the curve formed by slicing the ellipsoid
     * perpendicular to the meridian at a given latitude. It is essential for
     * calculating how East-West velocity translates into a change in longitude.
     * @param latitude The geodetic latitude in **radians**.
     * @return The transverse radius of curvature in [m].
     */
    T normal_radius(T latitude) const;

   private:
    ReferenceEllipsoid<T> m_datum;
    GravityModel<T> m_gravity_model;
    MagneticFieldModel<T> m_magnetic_model;
};

/*==========================================================================*/
/*================== Pre-defined Earth Model Standards =====================*/
/*==========================================================================*/

/**
 * @brief A pre-configured Earth model based on the **WGS-84** standard.
 * @details This is the most widely used model and serves as the basis for the Global
 * Positioning System (GPS). It combines the `ReferenceEllipsoidWGS84` and the
 * `GravityModelWGS84`.
 */
template <typename T = double>
struct EarthModelWGS84 : public EarthModel<T> {
    /// @brief Constructs a WGS-84 Earth model with standard parameters.
    EarthModelWGS84() : EarthModel<T>(ReferenceEllipsoidWGS84<T>(), GravityModelWGS84<T>()) {}
};

}  // namespace refx

#include "internal/earth_model.hpp"

#endif /* _REFX_MODELS_EARTH_MODEL_EARTH_MODEL_ */
