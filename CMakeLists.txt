cmake_minimum_required(VERSION 3.16)

project(ego VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(ego
    src/a_star.cpp
    src/ego.cpp
    src/polynomial_spline.cpp
)
add_library(ego::ego ALIAS ego)

target_compile_features(ego PUBLIC cxx_std_14)
target_include_directories(ego
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(ego PUBLIC Eigen3::Eigen)

set_target_properties(ego PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME ego
)

option(EGO_BUILD_EXAMPLES "Build ego examples" ON)
option(EGO_BUILD_TESTS "Build ego tests" ON)

if(EGO_BUILD_EXAMPLES)
    add_executable(ego_callback_example examples/callback_example.cpp)
    target_link_libraries(ego_callback_example PRIVATE ego::ego)

    add_executable(ego_2d_example examples/ego_2d_example.cpp)
    target_link_libraries(ego_2d_example PRIVATE ego::ego)

    add_executable(ego_3d_example examples/ego_3d_example.cpp)
    target_link_libraries(ego_3d_example PRIVATE ego::ego)
endif()

if(EGO_BUILD_TESTS)
    enable_testing()
    add_executable(ego_standalone_test tests/standalone_test.cpp)
    target_link_libraries(ego_standalone_test PRIVATE ego::ego)
    add_test(NAME ego_standalone_test COMMAND ego_standalone_test)
endif()

install(TARGETS ego
    EXPORT egoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(PROGRAMS scripts/plot_trajectory.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    RENAME ego_plot_trajectory
)
install(FILES README.md README.zh-CN.md
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/egoConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/egoConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/egoConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/ego"
)
install(EXPORT egoTargets
    FILE egoTargets.cmake
    NAMESPACE ego::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/ego"
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/egoConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/egoConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/ego"
)
