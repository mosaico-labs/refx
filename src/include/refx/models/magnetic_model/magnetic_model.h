#ifndef _REFX_MODELS_MAGNETIC_MODEL_MAGNETIC_MODEL_
#define _REFX_MODELS_MAGNETIC_MODEL_MAGNETIC_MODEL_

#include <string>
#include <vector>

#include "../../geometry/coordinate.h"
#include "../../geometry/vector.h"

/**
 * @file magnetic_model.h
 * @brief Defines the MagneticFieldModel class for modeling the Earth's magnetic field.
 * @details This file provides the `refx::MagneticFieldModel` interface and a concrete
 * implementation based on the World Magnetic Model (WMM). This model is essential for
 * applications that use a magnetometer to determine heading. It provides a reference
 * magnetic field vector for a given location and time, which can be compared against
 * sensor readings to estimate the vehicle's yaw angle relative to magnetic North.
 */

namespace refx {

/**
 * @brief An interface for modeling the Earth's magnetic field.
 * @tparam T The scalar type for all calculations (e.g., `float`, `double`).
 *
 * @details This class serves as the base for all magnetic field models. Its primary
 * role is to define a common interface for computing the magnetic field vector at any
 * point on or above the Earth.
 *
 * The default implementation represents a "null" model, where the magnetic field is
 * always zero. This allows the `MagneticFieldModel` to be an optional component in the
 * main `EarthModel` class. For practical applications, a concrete implementation like
 * `MagneticFieldModelWMM` should be used.
 */
template <typename T = double>
struct MagneticFieldModel {
    /**
     * @brief Virtual destructor for polymorphism.
     */
    virtual ~MagneticFieldModel() = default;

    /**
     * @brief Computes the magnetic field vector at a specific location and time.
     * @details This is the core function of the interface. Concrete implementations
     * will use a mathematical model (like the WMM) to predict the field.
     * @param coord The geodetic coordinate (`LLA`) at which to compute the field.
     * @param decimal_year The time of the computation, expressed as a decimal year
     * (e.g., 2025.5 for mid-2025).
     * @return A `Vector3D<ned>` representing the magnetic field components
     * {North, East, Down} in [nanoTeslas, nT]. The default implementation
     * returns a zero vector.
     */
    virtual Vector3D<ned, T> get_field(const Coordinate3D<lla, T>& coord, T decimal_year) const {
        return Vector3D<ned, T>(0, 0, 0);
    }
};

/*==========================================================================*/
/*================== Pre-defined Magnetic Model Standards ==================*/
/*==========================================================================*/

/**
 * @brief A concrete implementation of the **World Magnetic Model (WMM)**.
 * @details This class implements the standard model used by NATO, the U.S. Department
 * of Defense, and civilian navigation systems to describe the Earth's main magnetic
 * field. The model is produced in 5-year epochs (e.g., WMM2020, WMM2025).
 *
 * This implementation uses the **WMM2020** coefficients, which are valid from
 * 2020.0 to 2025.0. It provides a high-fidelity reference for the Earth's magnetic
 * field, which is critical for magnetometer calibration and heading estimation in
 * sensor fusion algorithms.
 *
 * @note The implementation is based on the official WMM2020 Technical Report from NOAA.
 */
template <typename T = double>
struct MagneticFieldModelWMM : public MagneticFieldModel<T> {
    /**
     * @brief Default constructor. Initializes the model with WMM2020 coefficients.
     */
    MagneticFieldModelWMM();

    /**
     * @brief Computes the magnetic field vector using the WMM2020 algorithm.
     * @details This function implements the standard WMM synthesis algorithm, which
     * involves evaluating a spherical harmonic expansion at the given location and time
     * to predict the magnetic field components.
     * @param coord The geodetic coordinate (`LLA`) at which to compute the field.
     * @param decimal_year The time of the computation (e.g., 2022.75). The WMM2020
     * model is valid for the range [2020.0, 2025.0).
     * @return A `Vector3D<ned>` representing the magnetic field components
     * {North (X), East (Y), Down (Z)} in [nanoTeslas, nT].
     */
    Vector3D<ned, T> get_field(const Coordinate3D<lla, T>& coord, T decimal_year) const override;

   private:
    // Internal state for the WMM calculation
    T m_epoch;
    T m_reference_radius_km;
    int m_max_degree;

    // Storage for Schmidt semi-normalized coefficients
    std::vector<std::vector<T>> c;   // Main field g(n,m) and h(n,m)
    std::vector<std::vector<T>> cd;  // Secular variation g_dot(n,m) and h_dot(n,m)
    std::vector<std::vector<T>> p;   // Associated Legendre polynomials
    std::vector<std::vector<T>> dp;  // Derivatives of Legendre polynomials
};

}  // namespace refx

// #include "internal/magnetic_model.hpp"

#endif /* _REFX_MODELS_MAGNETIC_MODEL_MAGNETIC_MODEL_ */
