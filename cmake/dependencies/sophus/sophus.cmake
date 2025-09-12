set(SOPHUS_USE_BASIC_LOGGING
    ON
    CACHE BOOL "Don't use fmt for Sophus library")
set(BUILD_SOPHUS_TESTS
    OFF
    CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES
    OFF
    CACHE BOOL "Don't build Sophus Examples")
set(BUILD_PYTHON_BINDINGS
    OFF
    CACHE BOOL "Don't build Sophus Python Bindings")

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.28")
  FetchContent_Declare(
    Sophus
    URL https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz
        SYSTEM EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    Sophus
    URL https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz)
  include("${CMAKE_CURRENT_LIST_DIR}/../../MakeDummyFindPackage.cmake")
  make_dummy_find_package(Sophus)
endif()

FetchContent_MakeAvailable(Sophus)
