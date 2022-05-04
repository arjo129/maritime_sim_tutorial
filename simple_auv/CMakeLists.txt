cmake_minimum_required(VERSION 3.5)

project(simple_auv)

#============================================================================
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-transport11 REQUIRED)

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)

#============================================================================
# Unit tests
include(CTest)
if(BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)

  add_executable(test_stationary tests/test_stationary.cc)
  target_include_directories(test_stationary
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_stationary
    PUBLIC
    ignition-gazebo6::ignition-gazebo6
    ignition-math6::ignition-math6
    ignition-transport11::ignition-transport11
    gtest
    gtest_main
  )
  gtest_discover_tests(test_stationary)
endif()


#============================================================================
install(DIRECTORY
  models
  worlds
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})
