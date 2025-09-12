function(make_dummy_find_package DEP_NAME)
  set(dummy_dir "${CMAKE_BINARY_DIR}/cmake/dummy-configs/${DEP_NAME}")
  file(MAKE_DIRECTORY "${dummy_dir}")

  set(dummy_config_file "${dummy_dir}/${DEP_NAME}Config.cmake")

  if(NOT EXISTS "${dummy_config_file}")
    file(WRITE "${dummy_config_file}" "set(${DEP_NAME}_FOUND TRUE)\n")
  endif()

  # Add dummy config directory to CMAKE_PREFIX_PATH once
  list(FIND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/cmake/dummy-configs" idx)
  if(idx EQUAL -1)
    list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/cmake/dummy-configs")
    set(CMAKE_PREFIX_PATH
        "${CMAKE_PREFIX_PATH}"
        CACHE STRING "Add dummy config path" FORCE)
  endif()
endfunction()
