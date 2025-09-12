if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.28")
  FetchContent_Declare(
    Bonxai
    GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
    GIT_TAG 47ffd0a2917c899f6199dfa71445481164298006
    SOURCE_SUBDIR
    bonxai_core
    SYSTEM
    EXCLUDE_FROM_ALL
    OVERRIDE_FIND_PACKAGE)
else()
  # Minimal flags for compatibility with ROS Humble (CMake 3.22)
  FetchContent_Declare(
    Bonxai
    GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
    GIT_TAG 47ffd0a2917c899f6199dfa71445481164298006
    SOURCE_SUBDIR bonxai_core)
  include("${CMAKE_CURRENT_LIST_DIR}/../../MakeDummyFindPackage.cmake")
  make_dummy_find_package(Bonxai)
endif()

FetchContent_MakeAvailable(Bonxai)
