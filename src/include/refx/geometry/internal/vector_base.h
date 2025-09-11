#ifndef _REFX_GEOMETRY_INTERNAL_VECTOR_BASE_
#define _REFX_GEOMETRY_INTERNAL_VECTOR_BASE_

#include <algorithm>  // for std::copy
#include <array>
#include <cmath>    // for std::sqrt
#include <cstddef>  // for size_t
#include <ostream>

// Conditionally include Eigen headers if support is enabled by the user.
#ifdef REFX_ENABLE_EIGEN_SUPPORT
#include <Eigen/Dense>
#endif

/**
 * @file vector_base.h
 * @brief Defines the low-level, non-frame-aware base classes for vector storage.
 * @details This internal header provides the foundational building blocks for all
 * vector and coordinate types in the library. These base classes (`VectorContainerXD`,
 * `VectorContainer3D`, `VectorContainer4D`) are responsible for the underlying data storage
 * (via `std::array`) and providing a minimal, generic interface.
 *
 * They are intentionally kept separate from the frame-aware logic to create a clean
 * architectural layer. All higher-level types, like `Vector3D<ned>`, inherit from
 * these classes to gain their storage and basic accessor capabilities.
 *
 * @section eigen_support Optional Eigen Interoperability
 * This library provides optional, compile-time interoperability with the Eigen linear
 * algebra library. To enable this feature, you must define the preprocessor macro
 * `REFX_ENABLE_EIGEN_SUPPORT` before including this header.
 * When enabled, all vector base classes will gain a constructor that accepts an
 * `Eigen::Matrix` and a `.to_eigen()` method for easy conversion.
 *
 * @warning These classes are implementation details and are not intended for direct
 * use by the end-user.
 */

namespace refx {
namespace internal {

/**
 * @brief A generic, fixed-size N-dimensional vector storage class.
 * @tparam D The dimension of the vector.
 * @tparam T The scalar type of the components (e.g., `float`, `double`).
 *
 * @details This class serves as the most fundamental building block for all vector
 * types. It is a thin wrapper around `std::array` that provides a common interface
 * for construction, element access, and basic scalar arithmetic. It has no knowledge
 * of coordinate frames or special mathematical semantics.
 */
template <std::size_t D, typename T>
struct VectorContainerXD {
    /// @brief The compile-time dimension of the vector.
    static constexpr std::size_t dim = D;
    /// @brief The underlying scalar type for the vector's components.
    using scalar_type = T;
    /// @brief The underlying container type used for storage.
    using container_type = std::array<T, D>;

    /// @brief The actual `std::array` that holds the vector's components.
    container_type data;

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /// @brief Type alias for the corresponding Eigen column vector type.
    using EigenVector = Eigen::Matrix<T, D, 1>;
#endif

    // --- Constructors ---

    /**
     * @brief Default constructor.
     * @warning The vector's components are left uninitialized for performance,
     * similar to fundamental types.
     */
    VectorContainerXD() = default;

    /**
     * @brief Fill constructor.
     * @details Initializes all components of the vector to a single scalar value.
     * @param value The value to assign to every component.
     */
    VectorContainerXD(const T& value) { data.fill(value); }

    /**
     * @brief Initializer list constructor.
     * @details Constructs the vector from a C++ initializer list.
     * @param init The initializer list. Its size should match the vector's dimension.
     */
    VectorContainerXD(std::initializer_list<T> init) {
        // Note: Assumes init.size() == D. No runtime check for performance.
        std::copy(init.begin(), init.end(), data.begin());
    }

    /**
     * @brief Copy constructor from an existing `std::array`.
     * @param v The `std::array` to copy from.
     */
    VectorContainerXD(const container_type& v) : data(v) {}

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Constructs a vector from an Eigen column vector.
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * It provides a way to convert from Eigen types to this library's types.
     * @param eigen_vec The `Eigen::Matrix<T, D, 1>` to construct from.
     */
    VectorContainerXD(const EigenVector& eigen_vec) {
        // Eigen's data is column-major by default, which is contiguous for vectors.
        std::copy(eigen_vec.data(), eigen_vec.data() + D, data.begin());
    }
#endif

    // --- Element Access ---

    /// @brief Provides mutable access to a component by its index.
    T& operator[](std::size_t i) { return data[i]; }
    /// @brief Provides const access to a component by its index.
    const T& operator[](std::size_t i) const { return data[i]; }

    // --- Iterators ---

    /// @brief Returns a mutable iterator to the beginning of the vector data.
    auto begin() { return data.begin(); }
    /// @brief Returns a mutable iterator to the end of the vector data.
    auto end() { return data.end(); }
    /// @brief Returns a const iterator to the beginning of the vector data.
    auto begin() const { return data.begin(); }
    /// @brief Returns a const iterator to the end of the vector data.
    auto end() const { return data.end(); }

    /// @brief Computes the L2 norm (magnitude) of the vector.
    T norm() const {
        T norm_sq = 0;
        for (const auto& elem : this->data) {
            norm_sq += elem * elem;
        }
        return std::sqrt(norm_sq);
    }

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Converts this vector to its equivalent Eigen column vector type.
     * @details This method is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * It uses `Eigen::Map` for an efficient conversion, typically avoiding a deep copy.
     * @return An `Eigen::Matrix<T, D, 1>` containing a copy of this vector's data.
     */
    EigenVector to_eigen() const {
        // `Eigen::Map` creates an Eigen object that wraps the existing std::array data.
        // Returning it by value triggers a copy into a new Eigen::Matrix.
        return Eigen::Map<const EigenVector>(data.data());
    }
#endif
};

/**
 * @brief Provides a convenient stream output operator for debugging vectors.
 * @details Formats the vector's components as `[c1, c2, ..., cD]`.
 */
template <std::size_t D, typename T>
std::ostream& operator<<(std::ostream& os, const VectorContainerXD<D, T>& v) {
    os << "[";
    for (std::size_t i = 0; i < D; i++) {
        os << v[i];
        if (i + 1 < D) os << ", ";
    }
    os << "]";
    return os;
}

/**
 * @brief A specialized base class for 3D vectors with named `x, y, z` accessors.
 * @tparam T The scalar type of the components.
 *
 * @details This class inherits from `VectorContainerXD<3, T>` and adds the standard
 * named accessors for 3D Cartesian components. It serves as the direct, non-frame-aware
 * base for all high-level `Vector3D` and `Coordinate3D` specializations, providing
 * them with the essential `.x()`, `.y()`, and `.z()` interface.
 */
template <typename T>
struct VectorContainer3D : public VectorContainerXD<3, T> {
    using VectorContainerXD<3, T>::VectorContainerXD;

    /**
     * @brief Constructs a 3D vector from its components.
     * @param x The first component (default: 0).
     * @param y The second component (default: 0).
     * @param z The third component (default: 0).
     */
    VectorContainer3D(T x = 0, T y = 0, T z = 0) : VectorContainerXD<3, T>({x, y, z}) {}

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Constructs a 3D vector from an Eigen 3D vector.
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * @param eigen_vec The `Eigen::Matrix<T, 3, 1>` to construct from.
     */
    VectorContainer3D(const typename VectorContainerXD<3, T>::EigenVector& eigen_vec)
        : VectorContainerXD<3, T>(eigen_vec) {}
#endif

    // --- Named Accessors ---
    /// @brief Provides mutable access to the first component (X).
    T& x() { return this->data[0]; }
    /// @brief Provides mutable access to the second component (Y).
    T& y() { return this->data[1]; }
    /// @brief Provides mutable access to the third component (Z).
    T& z() { return this->data[2]; }

    /// @brief Provides const access to the first component (X).
    const T& x() const { return this->data[0]; }
    /// @brief Provides const access to the second component (Y).
    const T& y() const { return this->data[1]; }
    /// @brief Provides const access to the third component (Z).
    const T& z() const { return this->data[2]; }
};

/**
 * @brief A specialized base class for 4D vectors with named `x, y, z, w` accessors.
 * @tparam T The scalar type of the components.
 *
 * @details This class inherits from `VectorContainerXD<4, T>` and adds standard named
 * accessors for 4D components. It is intended as a building block for higher-level
 * types such as quaternions or homogeneous coordinates.
 */
template <typename T>
struct VectorContainer4D : public VectorContainerXD<4, T> {
    using VectorContainerXD<4, T>::VectorContainerXD;

    /**
     * @brief Constructs a 4D vector from its components.
     * @param x The first component (default: 0).
     * @param y The second component (default: 0).
     * @param z The third component (default: 0).
     * @param w The fourth component (default: 0).
     */
    VectorContainer4D(T x = 0, T y = 0, T z = 0, T w = 0) : VectorContainerXD<4, T>({x, y, z, w}) {}

#ifdef REFX_ENABLE_EIGEN_SUPPORT
    /**
     * @brief Constructs a 4D vector from an Eigen 4D vector.
     * @details This constructor is only enabled if `REFX_ENABLE_EIGEN_SUPPORT` is defined.
     * @param eigen_vec The `Eigen::Matrix<T, 4, 1>` to construct from.
     */
    VectorContainer4D(const typename VectorContainerXD<4, T>::EigenVector& eigen_vec)
        : VectorContainerXD<4, T>(eigen_vec) {}
#endif

    // --- Named Accessors ---
    /// @brief Provides mutable access to the first component (X).
    T& x() { return this->data[0]; }
    /// @brief Provides mutable access to the second component (Y).
    T& y() { return this->data[1]; }
    /// @brief Provides mutable access to the third component (Z).
    T& z() { return this->data[2]; }
    /// @brief Provides mutable access to the fourth component (W).
    T& w() { return this->data[3]; }

    /// @brief Provides const access to the first component (X).
    const T& x() const { return this->data[0]; }
    /// @brief Provides const access to the second component (Y).
    const T& y() const { return this->data[1]; }
    /// @brief Provides const access to the third component (Z).
    const T& z() const { return this->data[2]; }
    /// @brief Provides const access to the fourth component (W).
    const T& w() const { return this->data[3]; }
};
}  // namespace internal
}  // namespace refx

#endif /* _REFX_GEOMETRY_INTERNAL_VECTOR_BASE_ */
