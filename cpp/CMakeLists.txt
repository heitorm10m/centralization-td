add_library(centraltd_core STATIC
    src/version.cpp
    src/well.cpp
    src/string_section.cpp
    src/centralizer.cpp
    src/bow_spring_geometry.cpp
    src/bow_spring_constitutive_model.cpp
    src/bow_spring_resultants.cpp
    src/trajectory_frame.cpp
    src/discretization.cpp
    src/centralizer_support.cpp
    src/lateral_equilibrium.cpp
    src/contact_approximation.cpp
    src/global_assembly.cpp
    src/global_solver.cpp
    src/vector_lateral_equilibrium.cpp
    src/vector_contact.cpp
    src/vector_global_assembly.cpp
    src/vector_global_solver.cpp
    src/mechanical_solver.cpp
    src/torque_drag.cpp
    src/coupling_driver.cpp
    src/solver_stub.cpp
)

target_include_directories(centraltd_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(centraltd_core PUBLIC cxx_std_20)
set_property(TARGET centraltd_core PROPERTY CXX_SCAN_FOR_MODULES OFF)

if(MSVC)
    target_compile_options(centraltd_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(centraltd_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS centraltd_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

if(CENTRALTD_BUILD_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    find_package(pybind11 CONFIG QUIET)

    if(NOT pybind11_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            pybind11
            URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_MakeAvailable(pybind11)
    endif()

    pybind11_add_module(centraltd_py MODULE bindings/pybind_module.cpp)
    target_link_libraries(centraltd_py PRIVATE centraltd_core)
    target_compile_features(centraltd_py PRIVATE cxx_std_20)
    set_property(TARGET centraltd_py PROPERTY CXX_SCAN_FOR_MODULES OFF)

    set_target_properties(centraltd_py PROPERTIES
        OUTPUT_NAME "_core"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/centraltd"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/centraltd"
    )

    install(TARGETS centraltd_py
        LIBRARY DESTINATION centraltd
        RUNTIME DESTINATION centraltd
    )
endif()

if(BUILD_TESTING AND CENTRALTD_BUILD_TESTS)
    add_executable(centraltd_smoke_test tests/test_smoke.cpp)
    target_link_libraries(centraltd_smoke_test PRIVATE centraltd_core)
    target_compile_features(centraltd_smoke_test PRIVATE cxx_std_20)
    set_property(TARGET centraltd_smoke_test PROPERTY CXX_SCAN_FOR_MODULES OFF)

    add_test(NAME centraltd_smoke_test COMMAND centraltd_smoke_test)
endif()
