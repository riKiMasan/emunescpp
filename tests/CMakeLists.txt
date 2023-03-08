add_executable(
  cpu_tests
  cpu_tests.cpp
)
target_link_libraries(
  cpu_tests
  GTest::gtest_main
  cpu
)

include(GoogleTest)
gtest_discover_tests(cpu_tests)
