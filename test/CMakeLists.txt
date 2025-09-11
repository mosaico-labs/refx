include(FetchContent)

# For other versions, see: https://github.com/google/googletest/releases
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

FetchContent_MakeAvailable(googletest)

include(GoogleTest)

include_directories("../include/")

# Add a test file to test suite
add_executable(test_example test_example.cpp)
target_link_libraries(test_example PRIVATE refx GTest::gtest_main)
gtest_discover_tests(test_example)

add_executable(doc_test_code doc_test_code/main.cpp)
target_link_libraries(doc_test_code PRIVATE refx)


