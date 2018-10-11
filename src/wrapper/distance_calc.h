//
// Created by david on 31.07.18.
//

#ifndef WRAPPER_DISTANCE_CALC_H
#define WRAPPER_DISTANCE_CALC_H

#include "detector-wrapper.hpp"

enum Distance_Strategy { CONE_HEIGHT = 0, CONE_WIDTH = 1, CONE_AVERAGE = 2 };

struct intrinsic_camera_parameters {

  // AOV calculated in
  // https://drive.google.com/open?id=1yfDZr2MJyqkEoziy0suvq2ejFtKxsR_Ukpk-Wdw1F5k
  // Degrees
  //const double h_AOV          = 75.3937;
  //const double v_AOV          = 63.5316;
  // RADIANS
  const double h_AOV;
  const double v_AOV;

  // Measurements in μm
  const double sensor_width;
  const double sensor_height;
  const double focal_length;

  intrinsic_camera_parameters()
  : h_AOV(1.3158683), v_AOV(1.1088353), sensor_width(6182.4), sensor_height(4953.6), focal_length(4210.0) 
  {}
};

// Cone measurements in μm
struct large_cone {
  const double width;
  const double height;

  large_cone()
  : width(285000.0), height(505000.0)
  {}
};

struct small_cone {
  const double width;
  const double height;

  small_cone()
  : width(228000.0), height(325000.0)
  {}
};

// ( image_width, image_size )
std::pair<const double, const double> image_size = {1280.0, 1048.0};

double get_bbox_angle ( const bbox_t & bbox )
{
  intrinsic_camera_parameters intrinsics = intrinsic_camera_parameters();
  const double image_width = std::get<0>(image_size);

  const double angle = (( bbox.x + bbox.w/2 ) / image_width ) * (- intrinsics.h_AOV) + ( intrinsics.h_AOV / 2);
  
  return angle;
}

double distance_estimation ( const bbox_t & bbox, Distance_Strategy distance_strategy )
{
  // Conversion from pixel height to height on sensor missing
  intrinsic_camera_parameters intrinsics = intrinsic_camera_parameters();
  double perpendicular_distance;
  double cone_width, cone_height;
  const double image_width  = std::get<0>(image_size);
  const double image_height = std::get<1>(image_size);
  
  const double width_on_sensor  = ( bbox.w / image_width )  * intrinsics.sensor_width;
  const double height_on_sensor = ( bbox.h / image_height ) * intrinsics.sensor_height;

  // Change real world dimensions for large cones
  if (bbox.obj_id > 2) {
    large_cone cone_dims = large_cone();
    cone_width  = cone_dims.width;
    cone_height = cone_dims.height;
  }
  else {
    small_cone cone_dims = small_cone();
    cone_width  = cone_dims.width;
    cone_height = cone_dims.height;
  }

  // Angles in Fahrzeugmodell-Koordinaten
  //const double angle = (( bbox.x + bbox.w/2 ) / image_width ) * (- intrinsics.h_AOV) + ( intrinsics.h_AOV / 2);
  const double angle = get_bbox_angle( bbox );

  switch ( distance_strategy ) {
    case CONE_HEIGHT:
      perpendicular_distance = ((cone_height * intrinsics.focal_length) / height_on_sensor) / 1000000.0;
      break;
    case CONE_WIDTH:
      perpendicular_distance = ((cone_width  * intrinsics.focal_length) / width_on_sensor) / 1000000.0;
      break;
  }

  const double distance = perpendicular_distance / std::cos( angle );
  return distance;
}

#endif //WRAPPER_DISTANCE_CALC_H
