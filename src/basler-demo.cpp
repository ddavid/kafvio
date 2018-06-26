#include <iostream>
#include <iomanip> 
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <boost/program_options.hpp> //Nice argparser

#ifdef _WIN32
#define OPENCV
#define GPU
#endif

// To use tracking - uncomment the following line. Tracking is supported only by OpenCV 3.x
#define TRACK_OPTFLOW

#define OPENCV
#define GPU

#include "yolo_v2_class.hpp"        // imported functions from DLL
//#include "object.hpp"             // Old Dirty, dirty FSD Object
#include "object.h"
#include <connector-1.0/client.h>   // Jussie & Alex: UDP/TCP Connector 

#include <pylon/PylonIncludes.h>    // Pylon SDK

#ifdef OPENCV
#include <opencv2/opencv.hpp>           // C++
#include "opencv2/core/version.hpp"
#ifndef CV_VERSION_EPOCH
#include "opencv2/videoio/videoio.hpp"
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)""CVAUX_STR(CV_VERSION_MINOR)""CVAUX_STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#ifdef TRACK_OPTFLOW
#pragma comment(lib, "opencv_cudaoptflow" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_cudaimgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#endif  // TRACK_OPTFLOW
#else
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_EPOCH)"" CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#endif  // CV_VERSION_EPOCH

namespace po = boost::program_options;

enum Distance_Strategy { CONE_HEIGHT = 0, CONE_WIDTH = 1 };

const int IMAGE_HEIGHT = 1024;
const int IMAGE_WIDTH  = 1280;

std::chrono::steady_clock::time_point steady_clara_start;

class track_kalman {
public:
    cv::KalmanFilter kf;
    int state_size, meas_size, contr_size;


    track_kalman(int _state_size = 10, int _meas_size = 10, int _contr_size = 0)
        : state_size(_state_size), meas_size(_meas_size), contr_size(_contr_size)
    {
        kf.init(state_size, meas_size, contr_size, CV_32F);

        cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-5));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1e-2));
        cv::setIdentity(kf.transitionMatrix);
    }

    void set(std::vector<bbox_t> result_vec) {
        for (size_t i = 0; i < result_vec.size() && i < state_size*2; ++i) {
            kf.statePost.at<float>(i * 2 + 0) = result_vec[i].x;
            kf.statePost.at<float>(i * 2 + 1) = result_vec[i].y;
        }
    }

    // Kalman.correct() calculates: statePost = statePre + gain * (z(k)-measurementMatrix*statePre);
    // corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    std::vector<bbox_t> correct(std::vector<bbox_t> result_vec) {
        cv::Mat measurement(meas_size, 1, CV_32F);
        for (size_t i = 0; i < result_vec.size() && i < meas_size * 2; ++i) {
            measurement.at<float>(i * 2 + 0) = result_vec[i].x;
            measurement.at<float>(i * 2 + 1) = result_vec[i].y;
        }
        cv::Mat estimated = kf.correct(measurement);
        for (size_t i = 0; i < result_vec.size() && i < meas_size * 2; ++i) {
            result_vec[i].x = estimated.at<float>(i * 2 + 0);
            result_vec[i].y = estimated.at<float>(i * 2 + 1);
        }
        return result_vec;
    }

    // Kalman.predict() calculates: statePre = TransitionMatrix * statePost;
    // predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    std::vector<bbox_t> predict() {
        std::vector<bbox_t> result_vec;
        cv::Mat control;
        cv::Mat prediction = kf.predict(control);
        for (size_t i = 0; i < prediction.rows && i < state_size * 2; ++i) {
            result_vec[i].x = prediction.at<float>(i * 2 + 0);
            result_vec[i].y = prediction.at<float>(i * 2 + 1);
        }
        return result_vec;
    }

};

class extrapolate_coords_t {
public:
    std::vector<bbox_t> old_result_vec;
    std::vector<float> dx_vec, dy_vec, time_vec;
    std::vector<float> old_dx_vec, old_dy_vec;

    void new_result(std::vector<bbox_t> new_result_vec, float new_time) {
        old_dx_vec = dx_vec;
        old_dy_vec = dy_vec;
        if (old_dx_vec.size() != old_result_vec.size()) std::cout << "old_dx != old_res \n";
        dx_vec = std::vector<float>(new_result_vec.size(), 0);
        dy_vec = std::vector<float>(new_result_vec.size(), 0);
        update_result(new_result_vec, new_time, false);
        old_result_vec = new_result_vec;
        time_vec = std::vector<float>(new_result_vec.size(), new_time);
    }

    void update_result(std::vector<bbox_t> new_result_vec, float new_time, bool update = true) {
        for (size_t i = 0; i < new_result_vec.size(); ++i) {
            for (size_t k = 0; k < old_result_vec.size(); ++k) {
                if (old_result_vec[k].track_id == new_result_vec[i].track_id && old_result_vec[k].obj_id == new_result_vec[i].obj_id) {
                    float const delta_time = new_time - time_vec[k];
                    if (abs(delta_time) < 1) break;
                    size_t index = (update) ? k : i;
                    float dx = ((float)new_result_vec[i].x - (float)old_result_vec[k].x) / delta_time;
                    float dy = ((float)new_result_vec[i].y - (float)old_result_vec[k].y) / delta_time;
                    float old_dx = dx, old_dy = dy;

                    // if it's shaking
                    if (update) {
                        if (dx * dx_vec[i] < 0) dx = dx / 2;
                        if (dy * dy_vec[i] < 0) dy = dy / 2;
                    } else {
                        if (dx * old_dx_vec[k] < 0) dx = dx / 2;
                        if (dy * old_dy_vec[k] < 0) dy = dy / 2;
                    }
                    dx_vec[index] = dx;
                    dy_vec[index] = dy;

                    //if (old_dx == dx && old_dy == dy) std::cout << "not shakin \n";
                    //else std::cout << "shakin \n";

                    if (dx_vec[index] > 1000 || dy_vec[index] > 1000) {
                        //std::cout << "!!! bad dx or dy, dx = " << dx_vec[index] << ", dy = " << dy_vec[index] << 
                        //  ", delta_time = " << delta_time << ", update = " << update << std::endl;
                        dx_vec[index] = 0;
                        dy_vec[index] = 0;                      
                    }
                    old_result_vec[k].x = new_result_vec[i].x;
                    old_result_vec[k].y = new_result_vec[i].y;
                    time_vec[k] = new_time;
                    break;
                }
            }
        }
    }

    std::vector<bbox_t> predict(float cur_time) {
        std::vector<bbox_t> result_vec = old_result_vec;
        for (size_t i = 0; i < old_result_vec.size(); ++i) {
            float const delta_time = cur_time - time_vec[i];
            auto &bbox = result_vec[i];
            float new_x = (float) bbox.x + dx_vec[i] * delta_time;
            float new_y = (float) bbox.y + dy_vec[i] * delta_time;
            if (new_x > 0) bbox.x = new_x;
            else bbox.x = 0;
            if (new_y > 0) bbox.y = new_y;
            else bbox.y = 0;
        }
        return result_vec;
    }

};

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
    boxes.erase(std::remove_if(
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
        }

        double perpendicular_distance;
        double straight_distance;

        // Angles in FZModell-Koordinaten
        double angle     = (( box.x + box.w/2 ) / image_width ) * (- h_AOV) + ( h_AOV / 2);


        if(box.obj_id > 2) perpendicular_distance = ((large_cone_dimension * focal_length) / dimension_on_sensor) / 1000000.0;
        else  perpendicular_distance = ((small_cone_dimension * focal_length) / dimension_on_sensor) / 1000000.0;

        straight_distance = perpendicular_distance / std::cos( angle );

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
#endif  // OPENCV


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
        
        for (auto &i : result_vec) 
        {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y 
                  << ", w = " << i.w << ", h = " << i.h
              << std::setprecision(3) << ", prob = " << i.prob 
                      << ", distance_width = " << width_objects.element[count].distance << std::setprecision(6) 
                      << ", distance_height = " << height_objects.element[count].distance
                      << ", angle = " << height_objects.element[count].angle << '\n';
            count++;
    }
        
        //free(width_objects);
        //free(height_objects);
        
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

    //free(objects);    
}

void send_objects_udp(std::vector<bbox_t> const result_vec, connector::client< connector::UDP > & sender, Distance_Strategy strat, double distance_threshold) {
    
    object_list_t  objects = bbox_into_object_list( result_vec, strat, distance_threshold );
    
    sender.send_udp< uint32_t >( objects.size );
    sender.send_udp< object_t >( objects.element[0], objects.size * sizeof( object_t ));
    sender.send_udp< double   >( objects.time_s );
    //free(objects);    
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

/*double GetFrameRate(Pylon::CInstantCamera cam)
{
    if (GenApi::IsAvailable(cam.GetNodeMap().GetNode("ResultingFrameRateAbs")))
    {
        return GenApi::CFloatPtr(cam.GetNodeMap().GetNode("ResultingFrameRateAbs"))->GetValue();
    }
    else return GenApi::CFloatPtr(cam.GetNodeMap().GetNode("ResultingFrameRate"))->GetValue(); // BCON and USB use SFNC3 names
}*/

int main(int argc, char *argv[])
{
    std::string       names_file;
    std::string       cfg_file;
    std::string       weights_file;
    std::string       filename;
    std::string       pfs_file_name;
    std::string       ip;
    int               pylon;
    int               record_stream;
    int               live_demo;
    int               valid_test;
    int               port;
    int               udp_test;
    int               tcp_test;
    float             thresh;
    int               undistort;
    int               tracking;
    int               strategy_index;

    double            distance_threshold;
    long              frame_counter = 0;

    Distance_Strategy distance_strategy;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("classes", po::value<std::string>(&names_file)->default_value("/home/nvidia/Documents/github-repos/darknet-cvAlexey/data/cones.names"), ".txt or .list file with one class name per line")
        ("config", po::value<std::string>(&cfg_file)->default_value("/home/nvidia/Documents/github-repos/darknet-cvAlexey/cfg/yolov3-hires.cfg"),  "Darknet .cfg file")
        ("weights", po::value<std::string>(&weights_file)->default_value("/home/nvidia/Documents/github-repos/darknet-cvAlexey/weights/yolov3-hires_20000.weights"), "Darknet .weights file")
        ("image_file", po::value<std::string>(&filename), "Single Image file to run detection on")
        ("list_file", po::value<std::string>(&filename), "List file of image paths to run detections on")
        ("video_file", po::value<std::string>(&filename), "Single Video file to run detection on")
        ("basler", po::value<int>(&pylon)->default_value(1), "Run demo with Basler Cam if set to 1")
        ("pfs_file", po::value<std::string>(&pfs_file_name)->default_value("../cam_config_120fps_auto-balance.pfs"), "Camera Config File to load.")
        ("record", po::value<int>(&record_stream)->default_value(0), "Record openCV stream to .avi file")
        ("live_demo", po::value<int>(&live_demo)->default_value(0), "Show openCV stream")
        ("thresh", po::value<float>(&thresh)->default_value(0.20), "Set probability threshold for detection")
        ("undistort", po::value<int>(&undistort)->default_value(0), "Set undistortion flag")
        ("tracking", po::value<int>(&tracking)->default_value(1), "Set tracking flag")
        ("valid_test", po::value<int>(&valid_test)->default_value(0), "Set to write detections from -list_file to image files in cwd")
        ("port", po::value<int>(&port)->default_value(4401), "Set port to send objects to")
        ("ip", po::value<std::string>(&ip)->default_value("127.0.0.1"), "Set ip to send objects to, default is localhost via FSD::Connector")
        ("udp_test", po::value<int>(&udp_test)->default_value(1), "If true, sends objects to specified ip:port")
        ("tcp_test", po::value<int>(&tcp_test)->default_value(0), "If true, sends objects to specified ip:port")
        ("distance_strategy", po::value<int>(&strategy_index)->default_value(0), "Sets the distance estimation strategy to be used.\n0 := Height\n1 := Width")
        ("distance_threshold", po::value<double>(&distance_threshold)->default_value(20.0), "Sets the distance threshold value over which no detections are forwarded")

    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
 
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Set Distance Estimation
    if      ( strategy_index == 0 ) distance_strategy = Distance_Strategy::CONE_HEIGHT;
    else if ( strategy_index == 1 ) distance_strategy = Distance_Strategy::CONE_WIDTH;
   
    std::cout << "Using Distance Strat: " << distance_strategy << std::endl;

    // create a 3x3 double-precision camera matrix
    cv::Mat cam_mtx  = (cv::Mat_<double>(3,3)
                                        << 862.47564274, 0.0         , 692.5289645
                                          , 0.0        , 851.02364963, 528.42008433
                                          , 0.0        , 0.0         , 1.0);
    cv::Mat dist_mtx = (cv::Mat_<double>(1,5) << -0.25596272,  0.14915701,  0.00074994, -0.001857  , -0.05665815);
    cv::Mat rvec     = (cv::Mat_<double>(3,1) << -0.05461275,  1.12613537,  0.04233885);
    cv::Mat tvec     = (cv::Mat_<double>(3,1) << -1.65275017, -2.34888592,  13.89459771);

    cv::Mat map1, map2, identity_mtx;
    cv::setIdentity(identity_mtx);

    cv::Size imageSize( IMAGE_HEIGHT, IMAGE_WIDTH );
    // Get New Camera Matrix
    // (Old camera matrix, distortion coefficients, img_size, what to do with empty pixels, new_img_size, valid_pixels_roi, keep_center_principal_point)
    cv::Mat new_cam_mtx = cv::getOptimalNewCameraMatrix(cam_mtx, dist_mtx, imageSize, 1, imageSize, 0, true);
    cv::initUndistortRectifyMap(cam_mtx, dist_mtx, identity_mtx, cam_mtx, imageSize, CV_16SC2, map1, map2);

    std::cout << "Trying to initialize Pylon" << std::endl;
    // Initialize Pylon
    Pylon::PylonAutoInitTerm autoInitTerm;
    std::cout << "Pylon successfully initialized" << std::endl;

    Pylon::CInstantCamera camera;
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;                
    Pylon::CPylonImage pylonImage;
   
    Pylon::CGrabResultPtr ptrGrabResult;

    std::cout << "Pylon Camera Instance and GrabPointer Created" << std::endl;    
     
    if ( pylon )
    {
        try
        {
          std::cout << "Trying to Attach Cam" << std::endl;
          camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
          std::cout << "Camera Attached.\n" << "Using Device: " << camera.GetDeviceInfo().GetModelName() << std::endl;
         
          camera.StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImageOnly, Pylon::EGrabLoop::GrabLoop_ProvidedByUser);        
          std::cout << "Grabbing Started" << std::endl;                  

          if ( pfs_file_name != "" )
          {
            Pylon::CFeaturePersistence::Load(pfs_file_name.c_str(), &camera.GetNodeMap(), true);
            std::cout << "Camera Config loaded: " << pfs_file_name << std::endl;
          }
        }
        catch ( Pylon::GenericException &e) { std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl; return 1; }

    }

    Detector detector(cfg_file, weights_file);

    steady_clara_start = std::chrono::steady_clock::now();

    auto obj_names = objects_names_from_file(names_file);
        
    std::string out_videofile = "/home/nvidia/Documents/github-repos/modi-scheduler/darknet-recording.avi";

    connector::client< connector::TCP > tcp_sender( port, ip );
    connector::client< connector::UDP > udp_sender( port, ip );
    if ( udp_test ) udp_sender.init();
    else if ( tcp_test) tcp_sender.init();
        
#ifdef TRACK_OPTFLOW
    Tracker_optflow tracker_flow;
    detector.wait_stream = true;
#endif

    while (true) 
    {   

        if(!pylon)
        {

        std::cout << "input image or video filename: ";
        if(filename.size() == 0) std::cin >> filename; std::cout << "File Name: " << filename << "\n";
        if (filename.size() == 0) break;

        try {
#ifdef OPENCV
            extrapolate_coords_t extrapolate_coords;
            bool extrapolate_flag = false;
            float cur_time_extrapolate = 0, old_time_extrapolate = 0;
            preview_boxes_t large_preview(100, 150, false), small_preview(50, 50, true);
            bool show_small_boxes = false;

            std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
            std::string const protocol = filename.substr(0, 7);
            if (file_ext == "avi" || file_ext == "mp4" || file_ext == "mjpg" || file_ext == "mov" ||    // video file
                protocol == "rtmp://" || protocol == "rtsp://" || protocol == "http://" || protocol == "https:/")   // video network stream
            {
                cv::Mat cap_frame, cur_frame, det_frame, write_frame;
                std::queue<cv::Mat> track_optflow_queue;
                int passed_flow_frames = 0;
                std::shared_ptr<image_t> det_image;
                std::vector<bbox_t> result_vec, thread_result_vec;
                detector.nms = 0.02;    // comment it - if track_id is not required
                std::atomic<bool> consumed, videowrite_ready;
                bool exit_flag = false;
                consumed = true;
                videowrite_ready = true;
                std::atomic<int> fps_det_counter, fps_cap_counter, tracking_counter;
                fps_det_counter  = 0;
                fps_cap_counter  = 0;
                tracking_counter = 0;
                int current_det_fps = 0, current_cap_fps = 0, current_tracking_fps = 0;
                std::thread t_detect, t_cap, t_videowrite;
                std::mutex mtx;
                std::condition_variable cv_detected, cv_pre_tracked;
                std::chrono::steady_clock::time_point steady_start, steady_end;
                cv::VideoCapture cap(filename); cap >> cur_frame;
                int const video_fps = cap.get(CV_CAP_PROP_FPS);
                cv::Size const frame_size = cur_frame.size();
                cv::VideoWriter output_video;
                // Stream Recording
                if ( record_stream ) output_video.open(out_videofile, CV_FOURCC('M', 'J', 'P', 'G'), std::max(35, video_fps), frame_size, true);

                while (!cur_frame.empty()) 
                {
                    // always sync
                    if (t_cap.joinable()) 
                    {
                        t_cap.join();
                        ++fps_cap_counter;
                        cur_frame = cap_frame.clone();
                    }
                    t_cap = std::thread([&]() 
                        { 
                          cv::Mat temp;
                          cv::Mat undistort_img;
                          
                          cap >> temp;

                          if( undistort )
                          {
                              //cv::undistort(temp, undistort_img, cam_mtx, dist_mtx, cam_mtx);
                              cv::remap(temp, undistort_img, map1, map2, cv::INTER_LINEAR);
                              cap_frame = temp;
                          }
                          else 
                          {
                              cap >> cap_frame;
                          }    
                        });
                    ++cur_time_extrapolate;

                    // swap result bouned-boxes and input-frame
                    if(consumed)
                    {
                        std::unique_lock<std::mutex> lock(mtx);
                        det_image = detector.mat_to_image_resize(cur_frame);
                        auto old_result_vec = detector.tracking_id(result_vec);
                        auto detected_result_vec = thread_result_vec;
                        result_vec = detected_result_vec;
    
                        if(tracking)
                        {
                          // track optical flow
                          if (track_optflow_queue.size() > 0) 
                          {
                            cv::Mat first_frame = track_optflow_queue.front();
                            tracker_flow.update_tracking_flow(track_optflow_queue.front(), result_vec);

                            while (track_optflow_queue.size() > 1) 
                            {
                                track_optflow_queue.pop();
                                result_vec = tracker_flow.tracking_flow(track_optflow_queue.front(), true);
                            }
                            track_optflow_queue.pop();
                            passed_flow_frames = 0;

                            result_vec = detector.tracking_id(result_vec);
                            auto tmp_result_vec = detector.tracking_id(detected_result_vec, false);
                            small_preview.set(first_frame, tmp_result_vec);

                            extrapolate_coords.new_result(tmp_result_vec, old_time_extrapolate);
                            old_time_extrapolate = cur_time_extrapolate;
                            extrapolate_coords.update_result(result_vec, cur_time_extrapolate - 1);
                          }
                        }
                        else
                        {
                          result_vec = detector.tracking_id(result_vec);  // comment it - if track_id is not required                 
                          extrapolate_coords.new_result(result_vec, cur_time_extrapolate - 1);
                        }
                        // add old tracked objects
                        for (auto &i : old_result_vec) 
                        {
                            auto it = std::find_if(result_vec.begin(), result_vec.end(),
                                [&i](bbox_t const& b) { return b.track_id == i.track_id && b.obj_id == i.obj_id; });
                            bool track_id_absent = (it == result_vec.end());
                            if (track_id_absent) 
                            {
                                if (i.frames_counter-- > 1)
                                    result_vec.push_back(i);
                            }
                            else 
                            {
                                it->frames_counter = std::min((unsigned)3, i.frames_counter + 1);
                            }
                        }
                        if(tracking)
                        {
                          tracker_flow.update_cur_bbox_vec(result_vec);
                          result_vec = tracker_flow.tracking_flow(cur_frame, true);   // track optical flow
                        }
                        consumed = false;
                        cv_pre_tracked.notify_all();
                    }
                    // launch thread once - Detection
                    if (!t_detect.joinable()) 
                    {
                        t_detect = std::thread([&]() 
                        {
                            auto current_image = det_image;
                            consumed = true;
                            while (current_image.use_count() > 0 && !exit_flag) 
                            {
                                auto result = detector.detect_resized(*current_image, frame_size.width, frame_size.height, 
                                    thresh, false); // true
                                ++fps_det_counter;
                                std::unique_lock<std::mutex> lock(mtx);
                                thread_result_vec = result;
                                consumed = true;
                                cv_detected.notify_all();
                                if (detector.wait_stream) 
                                {
                                    while (consumed && !exit_flag) cv_pre_tracked.wait(lock);
                                }
                                current_image = det_image;
                            }
                        });
                    }
                    //while (!consumed);    // sync detection
                    if (!cur_frame.empty()) 
                    {
                        steady_end = std::chrono::steady_clock::now();
                        if (std::chrono::duration<double>(steady_end - steady_start).count() >= 1) 
                        {
                            current_det_fps      = fps_det_counter;
                            current_cap_fps      = fps_cap_counter;
                            current_tracking_fps = tracking_counter;
                            steady_start = steady_end;
                            fps_det_counter  = 0;
                            fps_cap_counter  = 0;
                            tracking_counter = 0;
                        }

                        large_preview.set(cur_frame, result_vec);

                        if(tracking)
                        {
                           ++passed_flow_frames;
                          track_optflow_queue.push(cur_frame.clone());
                          result_vec = tracker_flow.tracking_flow(cur_frame); // track optical flow
                          ++tracking_counter;
                          extrapolate_coords.update_result(result_vec, cur_time_extrapolate);
                          small_preview.draw(cur_frame, show_small_boxes);
                        }
                 
                        auto result_vec_draw = result_vec;
                        if (extrapolate_flag) {
                            result_vec_draw = extrapolate_coords.predict(cur_time_extrapolate);
                            cv::putText(cur_frame, "extrapolate", cv::Point2f(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(50, 50, 0), 2);
                        }

                        if      ( udp_test ) send_objects_udp( result_vec, udp_sender, distance_strategy, distance_threshold );
                        else if ( tcp_test ) send_objects_tcp( result_vec, tcp_sender, distance_strategy, distance_threshold );

                        std::cout << "Detection FPS: " << current_det_fps << "\n";
                        std::cout << "Capture   FPS: " << current_cap_fps << "\n";
                        std::cout << "Tracking FPS: " << current_tracking_fps << "\n";
                        //show_console_result(result_vec, obj_names);
                        // Make Results always be on top of Console
                        std::cout << "\033[2J";
                        std::cout << "\033[1;1H";
                        
                        if( live_demo || record_stream)  draw_boxes(cur_frame, result_vec_draw, obj_names, current_det_fps, current_cap_fps);

                        if( live_demo )
                        {
                            large_preview.draw(cur_frame);

                            cv::imshow("window name", cur_frame);
                            int key = cv::waitKey(1);   // 3 or 16ms
                            
                            if (key == 'f') show_small_boxes = !show_small_boxes;
                            if (key == 'p') while (true) if(cv::waitKey(100) == 'p') break;
                            if (key == 't') tracking = !tracking;
                            if (key == 'e') extrapolate_flag = !extrapolate_flag;
                            if (key == 27) { exit_flag = true; break; }
                        }
                            
                        if (output_video.isOpened() && videowrite_ready && record_stream) 
                        {
                          if (t_videowrite.joinable()) t_videowrite.join();
                          write_frame = cur_frame.clone();
                          videowrite_ready = false;
                          t_videowrite = std::thread([&]() 
                          { 
                            output_video << write_frame; videowrite_ready = true;
                          });
                        }
                     }
                    
                    if( record_stream && !tracking )
                    {
                      // wait detection result for video-file only (not for net-cam)
                      if (protocol != "rtsp://" && protocol != "http://" && protocol != "https:/") 
                      {
                        std::unique_lock<std::mutex> lock(mtx);
                        while (!consumed) cv_detected.wait(lock);
                      }
                    }
                    
                }
                exit_flag = true;
                if (t_cap.joinable()) t_cap.join();
                if (t_detect.joinable()) t_detect.join();
                if (t_videowrite.joinable()) t_videowrite.join();
                std::cout << "Video ended \n";
                break;
                return 0;
            }
            else if (file_ext == "txt")
                {   // list of image files
                std::cout << "Starting List File calculations" << std::endl;
                std::ifstream file(filename);
                if (!file.is_open()) std::cout << "File not found! \n";
                else
                    {  
                        for (std::string line; std::getline(file, line);)  //file >> line;)
                        {

                        // Easier splitting for Alex
                        std::cout << "#" << std::endl;
                        std::cout << line << std::endl;
                        cv::Mat mat_img = cv::imread(line);
                        cv::Mat undistort_img;

                        //cv::undistort(mat_img, undistort_img, cam_mtx, dist_mtx, cam_mtx);
                        std::vector<bbox_t> result_vec;                        

                        if(undistort)
                        {
                          cv::Mat tmp(mat_img);
                          cv::remap(tmp, undistort_img, map1, map2, cv::INTER_LINEAR);
                          result_vec = detector.detect(undistort_img); 
                        }
                        else result_vec = detector.detect(mat_img);

                        if      ( udp_test ) send_objects_udp( result_vec, udp_sender, distance_strategy, distance_threshold );
                        else if ( tcp_test ) send_objects_tcp( result_vec, tcp_sender, distance_strategy, distance_threshold );
                        
                        //show_console_result_distances( result_vec, obj_names );
                        show_console_result_CLARA_test( result_vec, obj_names );
                
                        if( valid_test ) 
                        {
                          frame_counter++;
                          std::ostringstream stringStream;
                          stringStream << std::setfill('0');
                          stringStream << "valid_frames/jo6_";
                          stringStream << std::setw(8) << std::to_string(frame_counter);
                          stringStream << ".jpg";
                          std::string filename = stringStream.str();
                          draw_boxes(mat_img, result_vec, obj_names, distance_threshold);
                          cv::imwrite( filename , mat_img);
                        } 
                    }
                    break;
                }
            }
            else 
            {  // image file
                std::cout << "Single image file" << std::endl;

                cv::Mat undistort_img;
                cv::Mat mat_img = cv::imread(filename);

                cv::undistort(mat_img, undistort_img, cam_mtx, dist_mtx, cam_mtx);
                std::vector<bbox_t> result_vec = detector.detect(undistort_img);
                //result_vec = detector.tracking_id(result_vec);    // comment it - if track_id is not required
                
                if(live_demo)
                { 
                    draw_boxes(undistort_img, result_vec, obj_names, distance_threshold); 
                    cv::imshow("window name", undistort_img);
                }

                if      ( udp_test ) send_objects_udp( result_vec, udp_sender, distance_strategy, distance_threshold );
                else if ( tcp_test ) send_objects_tcp( result_vec, tcp_sender, distance_strategy, distance_threshold );

                show_console_result_distances(result_vec, obj_names);

                if(live_demo) cv::waitKey(0);
            }
    #else
            //std::vector<bbox_t> result_vec = detector.detect(filename);

            auto img = detector.load_image(filename);
            std::vector<bbox_t> result_vec = detector.detect(img);
            detector.free_image(img);
            show_console_result(result_vec, obj_names);
    #endif          
        }
            catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
            catch (...) { std::cerr << "unknown exception \n"; getchar(); }
            filename.clear();
        }
        else
        {   
            try 
            {
                std::cout << "Pylon Loop" << std::endl;
                
                extrapolate_coords_t extrapolate_coords;
                bool extrapolate_flag = false;
                float cur_time_extrapolate = 0, old_time_extrapolate = 0;
                preview_boxes_t large_preview(100, 150, false), small_preview(50, 50, true);
                bool show_small_boxes = false;

                cv::Mat cap_frame, cur_frame, det_frame, write_frame;
                std::queue<cv::Mat> track_optflow_queue;
                int passed_flow_frames = 0;
                std::shared_ptr<image_t> det_image;
                std::vector<bbox_t> result_vec, thread_result_vec;
                detector.nms = 0.02;    // comment it - if track_id is not required
                std::atomic<bool> consumed, videowrite_ready;
                consumed = true;
                videowrite_ready = true;
                std::atomic<int> fps_det_counter, fps_cap_counter, tracking_counter;
                fps_det_counter  = 0;
                fps_cap_counter  = 0;
                tracking_counter = 0;
                int current_det_fps = 0, current_cap_fps = 0, cur_tracking_fps = 0;
                std::thread t_detect, t_cap, t_videowrite;
                std::mutex mtx;
                std::condition_variable cv_detected, cv_pre_tracked;
                std::chrono::steady_clock::time_point steady_start, steady_end;               
                if(camera.IsGrabbing())
                {
                    camera.RetrieveResult( 5000, ptrGrabResult, Pylon::ETimeoutHandling::TimeoutHandling_ThrowException);
                    if( ptrGrabResult->GrabSucceeded())
                    {
                        formatConverter.Convert(pylonImage, ptrGrabResult);
                        cv::Mat temp(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
                        cv::Mat undistort_img;

                        if( undistort )
                        {
                          //cv::undistort(temp, undistort_img, cam_mtx, dist_mtx, cam_mtx);
                          cv::remap(temp, undistort_img, map1, map2, cv::INTER_LINEAR);
                          cur_frame = undistort_img;
                        }
                        else 
                        {
                          cur_frame = temp;
                        }                                       
                        std::cout << "Cap Dims: " << ptrGrabResult->GetHeight() << " " << ptrGrabResult->GetWidth() << std::endl;
                    }
                }
                                    
                //cv::VideoCapture cap(filename); cap >> cur_frame;
                int const video_fps = 60; //cap.get(CV_CAP_PROP_FPS);
                                
                int cam_video_fps;
                if (GenApi::IsAvailable(camera.GetNodeMap().GetNode("ResultingFrameRateAbs")))
                {
                    cam_video_fps = (int) GenApi::CFloatPtr(camera.GetNodeMap().GetNode("ResultingFrameRateAbs"))->GetValue();
                }
                else cam_video_fps = (int) GenApi::CFloatPtr(camera.GetNodeMap().GetNode("ResultingFrameRate"))->GetValue(); // BCON and USB use SFNC3 names
                
                std::cout << "Cam FPS: " << cam_video_fps << std::endl;                                

                cv::Size const frame_size = cur_frame.size();
                std::cout << frame_size.width << " " << frame_size.height << std::endl;
                // Record Stream
                cv::VideoWriter output_video;
                if ( record_stream ) output_video.open(out_videofile, CV_FOURCC('M', 'J', 'P', 'G'), /*std::max(35, cam_video_fps)*/ 30, frame_size, true);

                while (!cur_frame.empty()) 
                {
                    // always sync
                    if (t_cap.joinable()) 
                    {
                        t_cap.join();
                        ++fps_cap_counter;
                        cur_frame = cap_frame.clone();
                    }
                    t_cap = std::thread([&]() 
                    { 
                        if(camera.IsGrabbing())
                        {
                            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::ETimeoutHandling::TimeoutHandling_ThrowException);
                            if( ptrGrabResult->GrabSucceeded())
                            {
                                formatConverter.Convert(pylonImage, ptrGrabResult);
                                cv::Mat temp(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

                                cv::Mat undistort_img;
                                if( undistort )
                                {
                                  //cv::undistort(temp, undistort_img, cam_mtx, dist_mtx, cam_mtx);
                                  cv::remap(temp, undistort_img, map1, map2, cv::INTER_LINEAR);
                                  cap_frame = undistort_img;
                                } 
                                else 
                                {
                                  cap_frame = temp;
                                }
                            }
                        }
                    });
                    ++cur_time_extrapolate;

                    // swap result bounded-boxes and input-frame
                    if(consumed)
                    {
                        std::unique_lock<std::mutex> lock(mtx);
                        det_image = detector.mat_to_image_resize(cur_frame);
                        auto old_result_vec = detector.tracking_id(result_vec);
                        auto detected_result_vec = thread_result_vec;
                        result_vec = detected_result_vec;
                        
                        if(tracking)
                        {
                          // track optical flow
                          if (track_optflow_queue.size() > 0) 
                          {
                            //std::cout << "\n !!!! all = " << track_optflow_queue.size() << ", cur = " << passed_flow_frames << std::endl;
                            cv::Mat first_frame = track_optflow_queue.front();
                            tracker_flow.update_tracking_flow(track_optflow_queue.front(), result_vec);

                            while (track_optflow_queue.size() > 1) {
                                track_optflow_queue.pop();
                                result_vec = tracker_flow.tracking_flow(track_optflow_queue.front(), true);
                            }
                            track_optflow_queue.pop();
                            passed_flow_frames = 0;

                            result_vec = detector.tracking_id(result_vec);
                            auto tmp_result_vec = detector.tracking_id(detected_result_vec, false);
                            small_preview.set(first_frame, tmp_result_vec);

                            extrapolate_coords.new_result(tmp_result_vec, old_time_extrapolate);
                            old_time_extrapolate = cur_time_extrapolate;
                            extrapolate_coords.update_result(result_vec, cur_time_extrapolate - 1);
                          }
                        }
                        else
                        {
                          result_vec = detector.tracking_id(result_vec);  // comment it - if track_id is not required                 
                          extrapolate_coords.new_result(result_vec, cur_time_extrapolate - 1);
                        }
                        // add old tracked objects
                        for (auto &i : old_result_vec) {
                            auto it = std::find_if(result_vec.begin(), result_vec.end(),
                                [&i](bbox_t const& b) { return b.track_id == i.track_id && b.obj_id == i.obj_id; });
                            bool track_id_absent = (it == result_vec.end());
                            if (track_id_absent) 
                            {
                                if (i.frames_counter-- > 1)
                                    result_vec.push_back(i);
                            }
                            else 
                            {
                                it->frames_counter = std::min((unsigned)3, i.frames_counter + 1);
                            }
                        }

                        if(tracking)
                        {
                          tracker_flow.update_cur_bbox_vec(result_vec);
                          result_vec = tracker_flow.tracking_flow(cur_frame, true);   // track optical flow
                        }

                        consumed = false;
                        cv_pre_tracked.notify_all();
                    }
                    // launch thread once - Detection
                    if (!t_detect.joinable()) 
                    {
                        t_detect = std::thread([&]() 
                        {
                            auto current_image = det_image;
                            consumed = true;
                            while (current_image.use_count() > 0) {
                                auto result = detector.detect_resized(*current_image, frame_size.width, frame_size.height, thresh, false);  // true
                                ++fps_det_counter;
                                std::unique_lock<std::mutex> lock(mtx);
                                thread_result_vec = result;
                                consumed = true;
                                cv_detected.notify_all();
                                if (detector.wait_stream) {
                                    while (consumed) cv_pre_tracked.wait(lock);
                                }
                                current_image = det_image;
                            }
                        });
                    }
                    //while (!consumed);    // sync detection

                    if (!cur_frame.empty()) 
                    {
                        steady_end = std::chrono::steady_clock::now();
                        if (std::chrono::duration<double>(steady_end - steady_start).count() >= 1) 
                        {
                            current_det_fps  = fps_det_counter;
                            current_cap_fps  = fps_cap_counter;
                            cur_tracking_fps = tracking_counter;
                            steady_start = steady_end;
                            fps_det_counter  = 0;
                            fps_cap_counter  = 0;
                            tracking_counter = 0;
                        }

                        large_preview.set(cur_frame, result_vec);
                        if (tracking)
                        {
                          ++passed_flow_frames;
                          track_optflow_queue.push(cur_frame.clone());

                          result_vec = tracker_flow.tracking_flow(cur_frame); // track optical flow
                          ++tracking_counter;
                        }

                        extrapolate_coords.update_result(result_vec, cur_time_extrapolate);
                        small_preview.draw(cur_frame, show_small_boxes);
                 
                        auto result_vec_draw = result_vec;
                        if (extrapolate_flag) 
                        {
                            result_vec_draw = extrapolate_coords.predict(cur_time_extrapolate);
                            cv::putText(cur_frame, "extrapolate", cv::Point2f(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(50, 50, 0), 2);
                        }

                        if      ( udp_test ) send_objects_udp( result_vec, udp_sender, distance_strategy, distance_threshold );
                        else if ( tcp_test ) send_objects_tcp( result_vec, tcp_sender, distance_strategy, distance_threshold );

                        std::cout << "Detection FPS: " << current_det_fps  << "\n";
                        std::cout << "Capture   FPS: " << current_cap_fps  << "\n";
                        std::cout << "Tracking  FPS: " << cur_tracking_fps << "\n";
                        show_console_result_distances( result_vec, obj_names );
                        
                        // Make Results always be on top of Console
                        std::cout << "\033[2J";
                        std::cout << "\033[1;1H";       

                        if ( live_demo || record_stream ) draw_boxes(cur_frame, result_vec_draw, obj_names, current_det_fps, current_cap_fps);
                        
                        if (  live_demo )
                        {

                            //cv::namedWindow( "OpenCV Display Window", CV_WINDOW_NORMAL);

                            large_preview.draw(cur_frame);

                            cv::imshow("OpenCV Display Window", cur_frame);
                            int key = cv::waitKey(1);   // 3 or 16ms
                            if (key == 'f') show_small_boxes = !show_small_boxes;
                            if (key == 't') tracking = !tracking;
                            if (key == 'p') while (true) if(cv::waitKey(100) == 'p') break;
                            if (key == 'e') extrapolate_flag = !extrapolate_flag;
                        }
                                                    

                        if (output_video.isOpened() && videowrite_ready && record_stream)
                        {
                            if (t_videowrite.joinable()) t_videowrite.join();
                            write_frame = cur_frame.clone();
                            videowrite_ready = false;
                            t_videowrite = std::thread([&]()
                            {
                                 output_video << write_frame; videowrite_ready = true;
                            });
                        }
                    }

                    // wait detection result for video-file only (not for net-cam)
                    if ( record_stream && !tracking ) 
                    {
                        std::unique_lock<std::mutex> lock(mtx);
                        while (!consumed) cv_detected.wait(lock);
                    }
                }
                if (t_cap.joinable()) t_cap.join();
                if (t_detect.joinable()) t_detect.join();
                if (t_videowrite.joinable()) t_videowrite.join();
                std::cout << "Video ended \n";
                
            }
        catch ( Pylon::GenericException &e) { std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl; return 1; }
        catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
        catch (...) { std::cerr << "unknown exception \n"; getchar(); }
        filename.clear();
        }
    }
    return 0;
}
