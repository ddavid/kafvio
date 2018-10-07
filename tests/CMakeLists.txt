# define all tests here
set(TESTS example-test.cpp)

#find_package(PkgConfig)
#pkg_search_module(Eigen3 REQUIRED eigen3)

#add_executable(kafi_test
#    catch.h
#    main.cpp
#    ${TESTS})

add_executable(kafvio-mouse-tracker
    kafvio-mouse-tracker.cpp)

add_executable(cv-mouse-tracker
    cv-mouse-tracker.cpp)

# add all used libraries
#target_link_libraries(${TEST_NAME})
target_link_libraries(kafvio-mouse-tracker ${OpenCV_LIBS})
target_link_libraries(cv-mouse-tracker ${OpenCV_LIBS})