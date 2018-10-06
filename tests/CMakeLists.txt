set(TEST_NAME kafi_tests)

# define all tests here
set(TESTS example-test.cpp)

#find_package(PkgConfig)
#pkg_search_module(Eigen3 REQUIRED eigen3)

add_executable(${TEST_NAME}
    catch.h
    main.cpp
    ${TESTS})

# add all used libraries
target_link_libraries(${TEST_NAME})