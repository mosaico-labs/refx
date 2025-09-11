#ifndef _REFX_MODELS_GRAVITY_MODEL_GRAVITY_MODEL_
#define _REFX_MODELS_GRAVITY_MODEL_GRAVITY_MODEL_

#include "../reference_ellipsoid/reference_ellipsoid.h"

/**
 * @file gravity_model.h
 * @brief Defines the GravityModel class for modeling the Earth's normal gravity field.
 * @details This file provides the `refx::GravityModel` class, which encapsulates the
 * mathematical formulas and physical constants used to compute the Earth's theoretical
 * "normal" gravity. Normal gravity is the gravity produced by a uniform rotating
 * ellipsoid, and it serves as the standard reference for inertial navigation systems.
 */

namespace refx {

/**
 * @brief Specifies the type of height correction to apply to a gravity calculation.
 * @details The magnitude of gravity decreases with altitude. This enum allows the user
 * to select the mathematical model used to account for this variation, which is
 * critical for applications operating at varying heights, such as UAVs.
 */
enum class GravityHeightCorrection {
    /**
     * @brief No height correction is applied.
     * Gravity is calculated on the surface of the reference ellipsoid and is only a
     * function of latitude. Suitable for ground-based applications at or near sea level.
     */
    None,
    /**
     * @brief A spherical free-air correction is applied.
     * A common and computationally efficient approximation that models the gravity
     * decrease with height. It is reasonably accurate for altitudes typical of
     * most robotics and aerospace applications.
     */
    FreeAir,
    /**
     * @brief A more precise ellipsoidal correction is applied (Moritz, 1980).
     * This model provides higher accuracy by accounting for the non-spherical
     * (ellipsoidal) shape of the Earth in its height correction term.
     */
    Ellipsoidal
};

/**
 * @brief A mathematical model of the Earth's normal gravity field.
 * @tparam T The scalar type for all calculations (e.g., `float`, `double`).
 *
 * @details This class provides the necessary functions to compute the theoretical
 * normal gravity at any given latitude and altitude. It is fundamentally tied to a
 * specific `ReferenceEllipsoid`, as the Earth's shape and rotation are the primary
 * factors that determine the local gravity vector.
 */
template <typename T = double>
struct GravityModel {
    /**
     * @brief Constructs a gravity model from its defining physical constants.
     * @param gamma_e The magnitude of normal gravity at the equator in [m/s²].
     * @param gamma_p The magnitude of normal gravity at the poles in [m/s²].
     * @param GM The gravitational constant of the Earth (mass × G) in [m³/s²].
     * @param datum The reference ellipsoid that this gravity field corresponds to.
     */
    GravityModel(T gamma_e, T gamma_p, T GM, const ReferenceEllipsoid<T>& datum);

    /**
     * @brief The default constructor is deleted to ensure that a gravity model is
     * always explicitly initialized with a complete and valid set of constants.
     */
    GravityModel() = delete;

    // --- Accessors for Defining Constants ---

    /// @brief Returns the normal gravity at the equator (`γe`) in [m/s²].
    T gamma_e() const { return m_gamma_e; }

    /// @brief Returns the normal gravity at the poles (`γp`) in [m/s²].
    T gamma_p() const { return m_gamma_p; }

    /// @brief Returns the Somigliana constant `k`, used in the normal gravity formula.
    T kappa() const { return m_somigliana_k; }

    /// @brief Returns the Earth's standard gravitational parameter `GM` in [m³/s²].
    T GM() const { return m_GM; }

    /// @brief Returns the reference ellipsoid associated with this gravity model.
    const ReferenceEllipsoid<T>& reference_ellipsoid() const { return m_datum; }

    // --- Core Gravity Computations ---

    /**
     * @brief Computes normal gravity on the ellipsoid surface as a function of latitude.
     * @details This function implements the closed-form **Somigliana formula**:
     * `γ(φ) = γe * (1 + k sin²φ) / sqrt(1 - e²sin²φ)`, which gives the theoretical
     * gravity at sea level for a given latitude `φ`.
     * @param latitude The geodetic latitude in **radians**.
     * @return The magnitude of the normal gravity vector in [m/s²].
     */
    T normal_gravity_on_ellipsoid(T latitude) const;

    /**
     * @brief Applies a specified height correction to a surface gravity value.
     * @details This function takes a pre-computed surface gravity value and adjusts
     * it for altitude using one of the models from `GravityHeightCorrection`.
     * @param gamma_phi The normal gravity at sea-level for the given latitude, `φ`, in [m/s²].
     * @param lat The geodetic latitude in **radians**.
     * @param alt The altitude above the reference ellipsoid in [m].
     * @param hc The height correction model to apply.
     * @return The corrected gravity magnitude at the specified altitude in [m/s²].
     */
    T apply_height(T gamma_phi, T lat, T alt, GravityHeightCorrection hc) const;

   private:
    T m_gamma_e;
    T m_gamma_p;
    T m_somigliana_k;
    T m_GM;
    ReferenceEllipsoid<T> m_datum;
};

/*==========================================================================*/
/*================== Pre-defined Gravity Model Standards ===================*/
/*==========================================================================*/

/**
 * @brief A gravity model based on the **WGS-84** standard.
 * @details This model provides the standard gravity constants associated with the
 * `ReferenceEllipsoidWGS84`.
 */
template <typename T = double>
struct GravityModelWGS84 : public GravityModel<T> {
    /// @brief Constructs a WGS-84 gravity model with its standard defining constants.
    GravityModelWGS84()
        : GravityModel<T>(9.7803253359, 9.8321849378, 3.986004418e14,
                          ReferenceEllipsoidWGS84<T>()) {}
};

/**
 * @brief A gravity model based on the **GRS-80** standard.
 * @details This model provides the standard gravity constants associated with the
 * `ReferenceEllipsoidGRS80`.
 */
template <typename T = double>
struct GravityModelGRS80 : public GravityModel<T> {
    /// @brief Constructs a GRS-80 gravity model with its standard defining constants.
    GravityModelGRS80()
        : GravityModel<T>(9.7803267715, 9.8321863685, 3.986005e14, ReferenceEllipsoidGRS80<T>()) {}
};

}  // namespace refx

#include "internal/gravity_model.hpp"

#endif /* _REFX_MODELS_GRAVITY_MODEL_GRAVITY_MODEL_ */
