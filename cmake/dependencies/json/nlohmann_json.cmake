if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.28")
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz
        SYSTEM EXCLUDE_FROM_ALL OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz)
  include("${CMAKE_CURRENT_LIST_DIR}/../../MakeDummyFindPackage.cmake")
  make_dummy_find_package(nlohmann_json)
endif()
