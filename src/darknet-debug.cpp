//#include "yolo_v2_class.hpp"        // imported functions from DLL
#include "opencv_utils.hpp"           // Split wrapper
#include "detector-wrapper.hpp"
#include "tracker.hpp"
#include "darknet-util.hpp"

#include "opencv_kalman.hpp"

#include "object.h"                 // municHMotorsport Object

#include <pylon/PylonIncludes.h>    // Pylon SDK

#include <opencv2/opencv.hpp>           // C++
#include "opencv2/core/version.hpp"
#include "opencv2/videoio/videoio.hpp"

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

  double image_width  = std::get<0>(image_size);
  double image_height = std::get<1>(image_size);

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
  ("distance_strategy", po::value<int>(&strategy_index)->default_value(0), "Sets the distance estimation strategy to be used.\n0 := Height\n1 := Width.\n2 := Average")
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
  else if ( strategy_index == 2 ) distance_strategy = Distance_Strategy::CONE_AVERAGE;

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

  cv::Size imageSize( image_height, iamge_width );
  // Get New Camera Matrix
  // (Old camera matrix, distortion coefficients, img_size, what to do with empty pixels, new_img_size, valid_pixels_roi, keep_center_principal_point)
  cv::Mat new_cam_mtx = cv::getOptimalNewCameraMatrix(cam_mtx, dist_mtx, imageSize, 1, imageSize, 0, true);
  cv::initUndistortRectifyMap(cam_mtx, dist_mtx, identity_mtx, cam_mtx, imageSize, CV_16SC2, map1, map2); 

  Detector detector(cfg_file, weights_file);

  steady_clara_start = std::chrono::steady_clock::now();

  auto obj_names = objects_names_from_file(names_file);

  std::string out_videofile = "/home/nvidia/Documents/github-repos/modi-scheduler/darknet-recording.avi";

  connector::client< connector::TCP > tcp_sender( port, ip );
  connector::client< connector::UDP > udp_sender( port, ip );

  if ( udp_test ) udp_sender.init();
  else if ( tcp_test) tcp_sender.init();

  Tracker_optflow tracker_flow;

  if ( tracking ) detector.wait_stream = true;

  while (true) 
  {   
    std::cout << "input image or video filename: ";
    if(filename.size() == 0) std::cin >> filename; std::cout << "File Name: " << filename << "\n";
    if (filename.size() == 0) break;

    try 
    {
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
            // Only Detections
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
    }
    catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
    catch (...) { std::cerr << "unknown exception \n"; getchar(); }
    filename.clear();
  }
  return 0;
}