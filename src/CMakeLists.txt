
add_library(refx INTERFACE)

if(NOT TARGET refx::refx)
    add_library(refx::refx ALIAS refx)
endif()

target_include_directories(refx
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(${REFX_ENABLE_EIGEN_DETECTION})
    # Check if eigen is installed and enable compatibility layer accordingly
    find_package(Eigen3 3.4 QUIET)

    if(Eigen3_FOUND)
        add_compile_definitions(REFX_ENABLE_EIGEN_SUPPORT)
        target_link_libraries(refx INTERFACE Eigen3::Eigen)

        message(STATUS "Found Eigen3 version ${EIGEN3_VERSION_STRING}, enabling Eigen support.")
    else()
        message(STATUS "Eigen3 not found. To enable optional Eigen interoperability, install Eigen3 or set Eigen3_DIR.")
    endif()
endif()


