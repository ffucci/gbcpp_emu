include(CTest)
include(GoogleTest)

enable_testing()

find_package(fmt)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(
    test_gbemu_lib
    main.cpp
    test_cpu_context.cpp
)

target_include_directories(test_gbemu_lib PUBLIC include)

target_link_libraries(
    test_gbemu_lib

    gbemu_lib
    fmt::fmt
    GTest::gtest_main
)

gtest_discover_tests(test_gbemu_lib)