#include "detector-wrapper.hpp"

#include <connector-1.0/client.h>   // Jussie & Alex: UDP/TCP Connector "

namespace po = boost::program_options;

enum Distance_Strategy { CONE_HEIGHT = 0, CONE_WIDTH = 1, CONE_AVERAGE = 2 };

const int IMAGE_HEIGHT = 1024;
const int IMAGE_WIDTH  = 1280;

std::chrono::steady_clock::time_point steady_clara_start;

object_list_t  bbox_into_object_list( std::vector<bbox_t> boxes, Distance_Strategy strat, double distance_threshold = 20.0 )
{
    // Dirty conversion before adjusting bbox_t

    const double image_height = 1024.0;
    const double image_width  = 1280.0;

    // AOV calculated in 
    // https://drive.google.com/open?id=1yfDZr2MJyqkEoziy0suvq2ejFtKxsR_Ukpk-Wdw1F5k
    // Degrees
    //const double h_AOV          = 75.3937;
    //const double v_AOV          = 63.5316;
    // RADIANS
    const double h_AOV         = 1.3158683; // 0.0229662;
    const double v_AOV         = 1.1088353; // 0.0193528;

    // All sensor and lens measures are in μm

    const double focal_length  = 4210.0;
    const double sensor_width  = 6182.4;
    const double sensor_height = 4953.6;

    // Cone measurements also in μm

    const double small_cone_width  = 228000.0;
    const double small_cone_height = 325000.0;

    const double large_cone_width  = 285000.0;
    const double large_cone_height = 505000.0; 

    int index = 0;

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

    //std::cout << "Calculating using Height" << std::endl;
    // Calculate Distances with cone height
    for( bbox_t& box : boxes )
    {
        double dimension_on_sensor;
        double small_cone_dimension;
        double large_cone_dimension;

        double snd_dimension_on_sensor;
        double snd_small_cone_dimension;
        double snd_large_cone_dimension;

        switch( strat )
        {
            case CONE_HEIGHT:
                dimension_on_sensor  = (box.h / image_height) * sensor_height;
                small_cone_dimension = small_cone_height;
                large_cone_dimension = large_cone_height;
            case CONE_WIDTH:
                dimension_on_sensor  = (box.w / image_width) * sensor_width;
                small_cone_dimension = small_cone_width;
                large_cone_dimension = large_cone_width;
            case CONE_AVERAGE:
                dimension_on_sensor  = (box.h / image_height) * sensor_height;
                snd_dimension_on_sensor  = (box.w / image_width) * sensor_width;

                small_cone_dimension = small_cone_height;
                snd_small_cone_dimension = small_cone_width;

                large_cone_dimension = large_cone_height;
                snd_large_cone_dimension = large_cone_width;
        }

        double perpendicular_distance;
        double snd_perpendicular_distance;

        double straight_distance;
        double snd_straight_distance;

        // Angles in FZModell-Koordinaten
        double angle     = (( box.x + box.w/2 ) / image_width ) * (- h_AOV) + ( h_AOV / 2);


        if(box.obj_id > 2)
        {
            perpendicular_distance = ((large_cone_dimension * focal_length) / dimension_on_sensor) / 1000000.0;
            if( strat == CONE_AVERAGE ) snd_perpendicular_distance = ((snd_large_cone_dimension * focal_length) / snd_dimension_on_sensor) / 1000000.0;
        }
        else
        {
            perpendicular_distance = ((small_cone_dimension * focal_length) / dimension_on_sensor) / 1000000.0;
            if( strat == CONE_AVERAGE ) snd_perpendicular_distance = ((snd_small_cone_dimension * focal_length) / snd_dimension_on_sensor) / 1000000.0;
        }

        //if (strat == CONE_AVERAGE) perpendicular_distance = ( perpendicular_distance + snd_perpendicular_distance ) / 2.0;

        straight_distance = perpendicular_distance / std::cos( angle );
        if (strat == CONE_AVERAGE)
        {
            snd_straight_distance = snd_perpendicular_distance / std::cos( angle );
            //std::cout << straight_distance << "\n" << snd_straight_distance << "\n";
            straight_distance = ( straight_distance + snd_straight_distance ) / 2;
        }

        if(straight_distance < distance_threshold)
        {
            object_t temp;
            temp.distance = straight_distance;
            temp.angle    = angle;
            temp.type     = box.obj_id;
            temp.x_car = 0;
            temp.y_car = 0;
            temp.angle_yaw = 0;
            temp.vx = 0;
            temp.vy = 0;
            temp.ax = 0;
            temp.ay = 0;
            temp.yaw_rate = 0;
            temp.steering_rad = 0;
            temp.time_s = clara_delta.count();

            tmp_objects.push_back(temp);
        }
        else
        {
            object_t temp;
            temp.distance = straight_distance;
            temp.angle    = angle;
            temp.type     = box.obj_id;
            temp.x_car = 0;
            temp.y_car = 0;
            temp.angle_yaw = 0;
            temp.vx = 0;
            temp.vy = 0;
            temp.ax = 0;
            temp.ay = 0;
            temp.yaw_rate = 0;
            temp.steering_rad = 0;
            temp.time_s = clara_delta.count();

            tmp_objects.push_back(temp);
        }
    }

    // Put object_t's into real list
    object_list_t cones;
    cones.size  = tmp_objects.size();
    cones.time_s = clara_delta.count();

    for(int i = 0; i < tmp_objects.size(); i++)
    {
        cones.element[i] = tmp_objects[i];
    }
    return cones;
}

// Put into opencv-utils after decoupling distance estimation and list creation
void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, double distance_threshold, int current_det_fps = -1, int current_cap_fps = -1)
{
    object_list_t height_objects = bbox_into_object_list( result_vec, Distance_Strategy::CONE_HEIGHT, distance_threshold);

    int index = 0;
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_caption = std::to_string(height_objects.element[index].distance) + " , angle: " + std::to_string(height_objects.element[index].angle);
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
        object_list_t  avg_objects = bbox_into_object_list(result_vec, Distance_Strategy::CONE_AVERAGE);

        for (auto &i : result_vec) 
        {
            if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y 
                  << ", w = " << i.w << ", h = " << i.h
                  << std::setprecision(3) << ", prob = " << i.prob
                  << std::setprecision(6) << ", distance_width = " << width_objects.element[count].distance
                  << ", distance_height = " << height_objects.element[count].distance
                  << ", avg_height = " << avg_objects.element[count].distance

                  << ", angle = " << height_objects.element[count].angle << '\n';
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

void send_objects_tcp(std::vector<bbox_t> const result_vec, connector::client< connector::TCP > & sender, Distance_Strategy strat, double distance_threshold) {
    
    object_list_t  objects = bbox_into_object_list( result_vec, strat, distance_threshold );
    
    sender.send_tcp< uint32_t >( objects.size );
    sender.send_tcp< object_t >( objects.element[0], objects.size * sizeof( object_t ));
    sender.send_tcp< double   >( objects.time_s );   
}

void send_objects_udp(std::vector<bbox_t> const result_vec, connector::client< connector::UDP > & sender, Distance_Strategy strat, double distance_threshold) {
    
    object_list_t  objects = bbox_into_object_list( result_vec, strat, distance_threshold );
    
    sender.send_udp< uint32_t >( objects.size );
    sender.send_udp< object_t >( objects.element[0], objects.size * sizeof( object_t ));
    sender.send_udp< double   >( objects.time_s );   
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}