set(EIGEN_BUILD_DOC
    OFF
    CACHE BOOL "Don't build Eigen docs")
set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "Don't build Eigen tests")
set(BUILD_TESTING
    OFF
    CACHE BOOL "Don't build Eigen tests but without the EIGEN prefix")
set(EIGEN_BUILD_PKGCONFIG
    OFF
    CACHE BOOL "Don't build Eigen pkg-config")
set(EIGEN_BUILD_BLAS
    OFF
    CACHE BOOL "Don't build blas module")
set(EIGEN_BUILD_LAPACK
    OFF
    CACHE BOOL "Don't build lapack module")

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.28")
  FetchContent_Declare(
    Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        SYSTEM EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
  include("${CMAKE_CURRENT_LIST_DIR}/../../MakeDummyFindPackage.cmake")
  make_dummy_find_package(Eigen3)
endif()
