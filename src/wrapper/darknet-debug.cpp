//#include "yolo_v2_class.hpp"        // imported functions from DLL
#include "opencv_utils.hpp"           // Split wrapper
#include "detector-wrapper.hpp"
#include "darknet-util.hpp"

#include "opencv_kalman.hpp"

#include "object.h"                   // municHMotorsport Object

#include <opencv2/opencv.hpp>         // C++
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "../odometry/odometry.h"
#include "../kalman/kalman_tracker.h"
#include "../kalman/kalman_config.h"

int main(int argc, char *argv[])
{   
  std::string       names_file;
  std::string       cfg_file;
  std::string       weights_file;
  std::string       filename;
  std::string       pfs_file_name;
  std::string       ip;
  int               record_stream;
  int               live_demo;
  int               valid_test;
  float             thresh;
  int               strategy_index;
  int               odom_state_dim = 2;
  int               odom_meas_dim  = 2;
  int               bbox_tracker_state_dim = 4;
  int               bbox_tracker_meas_dim  = 2;

  double            distance_threshold;
  long              frame_counter = 0;

  cpppc::Odometry_Filter_Matrices<double> odom_matrices;

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
  ("record", po::value<int>(&record_stream)->default_value(0), "Record openCV stream to .avi file")
  ("live_demo", po::value<int>(&live_demo)->default_value(0), "Show openCV stream")
  ("thresh", po::value<float>(&thresh)->default_value(0.20), "Set probability threshold for detection")
  ("valid_test", po::value<int>(&valid_test)->default_value(0), "Set to write detections from -list_file to image files in cwd")
  ("distance_strategy", po::value<int>(&strategy_index)->default_value(0), "Sets the distance estimation strategy to be used.\n0 := Height\n1 := Width.\n2 := Average")
  ("distance_threshold", po::value<double>(&distance_threshold)->default_value(30.0), "Sets the distance threshold value over which no detections are forwarded")

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

  Detector detector(cfg_file, weights_file);

  std::vector<bbox_t> empty_vec;
  cpppc::Odometry<2, 2> odometry_holder(empty_vec, odom_matrices.transition_matrix, odom_matrices.measurement_matrix);
  cpppc::BBox_Tracker<4, 2> kalman_tracker(empty_vec);

  steady_clara_start = std::chrono::steady_clock::now();

  auto obj_names = objects_names_from_file(names_file);

  std::string out_videofile = "/home/david/Videos/darknet-recording.avi";

  while (true) 
  {   
    std::cout << "input image or video filename: ";
    if(filename.empty()) std::cin >> filename; std::cout << "File Name: " << filename << "\n";
    if (filename.empty()) break;

    try 
    {
      preview_boxes_t large_preview(100, 150, false), small_preview(50, 50, true);
      bool show_small_boxes = true;

      std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
      std::string const protocol = filename.substr(0, 7);
      if (file_ext == "avi" || file_ext == "mp4" || file_ext == "mjpg" || file_ext == "mov" ||    // video file
      protocol == "rtmp://" || protocol == "rtsp://" || protocol == "http://" || protocol == "https:/")   // video network stream
      {
      cv::Mat cap_frame, cur_frame, det_frame, write_frame;
      std::shared_ptr<image_t> det_image;
      std::vector<bbox_t> result_vec, thread_result_vec;
      detector.nms = 0.02;    // comment it - if track_id is not required
      std::atomic<bool> consumed, videowrite_ready;
      bool exit_flag = false;
      consumed = true;
      videowrite_ready = true;
      std::atomic<int> fps_det_counter, fps_cap_counter;
      fps_det_counter  = 0;
      fps_cap_counter  = 0;
      int current_det_fps = 0, current_cap_fps = 0;
      std::thread t_detect, t_cap, t_videowrite;
      std::mutex mtx;
      std::condition_variable cv_detected, cv_pre_tracked;
      std::chrono::steady_clock::time_point steady_start, steady_end;
      cv::VideoCapture cap(filename); cap >> cur_frame;
      int const video_fps = cap.get(CV_CAP_PROP_FPS);
      cv::Size const frame_size = cur_frame.size();
      cv::VideoWriter output_video;

      double velocity_estimate, kafi_velocity_estimate;
      // Stream Recording
      if ( record_stream ) output_video.open(out_videofile, CV_FOURCC('M', 'J', 'P', 'G'), std::min(35, video_fps), frame_size, true);

        while (!cur_frame.empty()) 
        {
          // always sync
          if (t_cap.joinable()) 
          {
            t_cap.join();
            ++fps_cap_counter;
            cur_frame = cap_frame.clone();
          }
          // Get new frame
          t_cap = std::thread([&](){cap >> cap_frame;});

          // swap result bboxes and input-frame
          if(consumed)
          {
            // Try predicting more often...
            kalman_tracker.predict();

            std::unique_lock<std::mutex> lock(mtx);
            det_image = detector.mat_to_image_resize(cur_frame);
            auto old_result_vec = detector.tracking_id(result_vec);
            auto detected_result_vec = thread_result_vec;
            result_vec = detected_result_vec;

            // Only Detection
            result_vec = detector.tracking_id(result_vec);  // comment it - if track_id is not required

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

            //odometry_holder.update_bboxes(result_vec);
            velocity_estimate      = odometry_holder.calc_filtered_velocity(result_vec, 6.12);
            kalman_tracker.update_tracker(result_vec);
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
          while (!consumed){};    // sync detection
          if (!cur_frame.empty()) 
          {
            steady_end = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(steady_end - steady_start).count() >= 1) 
            {
              current_det_fps      = fps_det_counter;
              current_cap_fps      = fps_cap_counter;
              steady_start = steady_end;
              fps_det_counter  = 0;
              fps_cap_counter  = 0;
            }

            large_preview.set(cur_frame, result_vec);

            //std::cout << "Detection FPS: " << current_det_fps << "\n";
            //std::cout << "Capture   FPS: " << current_cap_fps << "\n";
            //show_console_result_distances( result_vec, obj_names );
            //show_console_result(result_vec, obj_names);
            // Make Results always be on top of Console
            //std::cout << "\033[2J";
            //std::cout << "\033[1;1H";

            // current_det_fps, current_cap_fps
            if( live_demo || record_stream)  draw_boxes(cur_frame, result_vec, obj_names, current_det_fps, velocity_estimate);

            if( live_demo )
            {
              large_preview.draw(cur_frame);

              cv::namedWindow("CPPPC Demo", cv::WINDOW_NORMAL);
              cv::imshow("CPPPC Demo", cur_frame);
              int key = cv::waitKey(1);
              if (key == 'f') show_small_boxes = !show_small_boxes;
              if (key == 'p') while (true) if(cv::waitKey(100) == 'p') break;
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

            if( record_stream )
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

            std::vector<bbox_t> result_vec;                        

            result_vec = detector.detect(mat_img);

            //show_console_result_distances( result_vec, obj_names );
            show_console_result_CLARA_test( result_vec, obj_names );

            if( valid_test ) 
            {
              frame_counter++;
              std::ostringstream stringStream;
              stringStream << std::setfill('0');
              stringStream << "cpppc-demo/accell-demo_";
              stringStream << std::setw(8) << std::to_string(frame_counter);
              stringStream << ".jpg";
              std::string valid_filename = stringStream.str();
              draw_boxes(mat_img, result_vec, obj_names);
              cv::imwrite( valid_filename , mat_img);
            } 
          }
          break;
        }
      }
      else 
      {  // image file
        std::cout << "Single image file" << std::endl;

        cv::Mat mat_img = cv::imread(filename);

        std::vector<bbox_t> result_vec = detector.detect(mat_img);
        //result_vec = detector.tracking_id(result_vec);    // comment it - if track_id is not required

        if(live_demo)
        { 
          draw_boxes(mat_img, result_vec, obj_names);
          cv::imshow("window name", mat_img);
        }

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
