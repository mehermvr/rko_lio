option(BUILD_SHARED_LIBS OFF)
option(TBBMALLOC_BUILD OFF)
option(TBB_EXAMPLES OFF)
option(TBB_STRICT OFF)
option(TBB_TEST OFF)

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.28")
  FetchContent_Declare(
    TBB
    URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.2.0.tar.gz
        SYSTEM
        EXCLUDE_FROM_ALL
        OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    TBB
    URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.2.0.tar.gz
  )
  include("${CMAKE_CURRENT_LIST_DIR}/../../MakeDummyFindPackage.cmake")
  make_dummy_find_package(TBB)
endif()

FetchContent_MakeAvailable(TBB)
