#include "catch.h"
#include "../src/kalman/kalman_filter.h"
#include <eigen3/Eigen/Dense>

TEST_CASE("simple-test", "[simple]")
{
  Kalman_Filter<1, 1, 1> kafi;

    //REQUIRE(( Eigen::Matrix<double, 1, 1>::Zero() == kafi.post_state ));
  REQUIRE(( Eigen::Matrix<double, 1, 1>::Zero() == kafi.post_state ));
}
