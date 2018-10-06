#include "detector-wrapper.hpp"
#include "distance_calc.h"

namespace po = boost::program_options;

std::chrono::steady_clock::time_point steady_clara_start;

object_t bbox_into_object( const bbox_t & bbox, const double distance )
{
  object_t obj;
    
  obj.distance  = distance;
  obj.angle     = get_bbox_angle( bbox );
  obj.type      = bbox.obj_id;
  
  // Default initialize
  obj.x_car     = 0;
  obj.y_car     = 0;
  obj.angle_yaw = 0;
  obj.vx        = 0;
  obj.vy        = 0;
  obj.ax        = 0;
  obj.ay        = 0;
  obj.yaw_rate  = 0;
  obj.steering_rad = 0;

  return obj;
}

object_list_t  bbox_into_object_list( std::vector<bbox_t> boxes, Distance_Strategy strat, double distance_threshold = 20.0 )
{
  //  Remove detections that clip boundaries of the screen
  /*boxes.erase(std::remove_if(
          boxes.begin(),
          boxes.end(),
          [image_width, image_height](bbox_t & box)
         {
             return
              box.x          <= 0            ||
             (box.x + box.w) >= image_width  ||
              box.y          <= 0            ||
             (box.y + box.h) >= image_height;
          }), boxes.end());
  */
  std::vector<object_t> tmp_objects;

  std::chrono::steady_clock::time_point steady_clara_measure = std::chrono::steady_clock::now();
  std::chrono::duration<double> clara_delta = steady_clara_measure - steady_clara_start;

  for ( bbox_t& bbox : boxes )
  {

    const double distance = distance_estimation( bbox, strat );

    if ( distance < distance_threshold )
    {
      object_t temp = bbox_into_object(bbox, distance);
      temp.time_s = clara_delta.count();
      tmp_objects.push_back(temp);
    }
  }

  // Put object_t's into real list
  object_list_t cones;
  cones.size  = tmp_objects.size();
  cones.time_s = clara_delta.count();

  for ( int i = 0; i < tmp_objects.size(); i++ )
  {
      cones.element[i] = tmp_objects[i];
  }
  return cones;
}

// Put into opencv-utils after decoupling distance estimation and list creation
void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, int current_det_fps = -1, int current_cap_fps = -1)
{
    //object_list_t height_objects = bbox_into_object_list( result_vec, Distance_Strategy::CONE_HEIGHT, distance_threshold);

    int index = 0;
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);

        if (obj_names.size() > i.obj_id) {
          std::string obj_caption;
          obj_caption = std::to_string(distance_estimation(i, Distance_Strategy::CONE_HEIGHT) ) + " , angle: " + std::to_string(get_bbox_angle(i) );

            if (i.track_id > 0) std::to_string(height_objects.element[index].distance) += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_caption, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, 0);
            int const max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 30, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols-1), std::min((int)i.y, mat_img.rows-1)),
                color, CV_FILLED, 8, 0);
            // Yellow captions in black
            if (i.obj_id == 0) putText(mat_img, obj_caption, cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
            else putText(mat_img, obj_caption, cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        }
        ++index;
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    }
}


void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names) {
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y 
            << ", w = " << i.w << ", h = " << i.h
            << std::setprecision(3) << ", prob = " << i.prob << std::endl;
    }
}

void show_console_result_distances(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names) {
    
        int count = 0;
        
        // Width distance estimation
        object_list_t  width_objects  = bbox_into_object_list(result_vec, Distance_Strategy::CONE_WIDTH);
        //Height distance estimation
        object_list_t  height_objects = bbox_into_object_list(result_vec, Distance_Strategy::CONE_HEIGHT);
        //Average distance estimation
        //object_list_t  avg_objects = bbox_into_object_list(result_vec, Distance_Strategy::CONE_AVERAGE);

        for (auto &i : result_vec) 
        {
            if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
            std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y 
                      << ", w = " << i.w << ", h = " << i.h
                      << std::setprecision(3) << ", prob = " << i.prob
                      << std::setprecision(6) << ", distance_width = " << width_objects.element[count].distance
                      << ", distance_height = " << height_objects.element[count].distance
                      << ", angle = " << height_objects.element[count].angle
                      << ", time_s = " << height_objects.element[count].time_s << '\n';
            count++;
    }     
}

void show_console_result_CLARA_test(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names)
{
    int count = 0;

    //Height distance estimation
    object_list_t height_objects = bbox_into_object_list(result_vec, Distance_Strategy::CONE_HEIGHT);

    for (auto &i : result_vec)
    {
        // Distance
        std::cout << height_objects.element[count].distance << std::setprecision(6)
        << "," <<
        // Id
        i.obj_id
        << "," <<
        // Angle
        height_objects.element[count].angle << std::endl;
        ++count;
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}
