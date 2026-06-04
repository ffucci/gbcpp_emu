include(GoogleTest)

enable_testing()

find_package(GTest REQUIRED CONFIG)

add_executable(
    test_gbemu_lib
    main.cpp
    test_cpu_context.cpp
    test_gamepad.cpp
)

target_include_directories(test_gbemu_lib PUBLIC include)

target_link_libraries(
    test_gbemu_lib
    PRIVATE
        gbemu_lib
        GTest::gtest_main
)

gtest_discover_tests(test_gbemu_lib)
