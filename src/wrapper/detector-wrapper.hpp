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

#ifndef YOLODLL_API
#define YOLODLL_API


struct bbox_t {
	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;					// confidence - probability that the object was found correctly
	unsigned int obj_id;		// class of object - from range [0, classes-1]
	unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
	unsigned int frames_counter;// counter of frames on which the object was detected
};

struct image_t {
	int h;						// height
	int w;						// width
	int c;						// number of chanels (3 - for RGB)
	float *data;				// pointer to the image data
};


#include "object.h"

#ifdef __cplusplus
#include <memory>
#include <vector>
#include <deque>
#include <algorithm>
#include <boost/cstdint.hpp>

#include <opencv2/opencv.hpp>			// C++
#include "opencv2/highgui/highgui_c.h"	// C
#include "opencv2/imgproc/imgproc_c.h"	// C

class Detector {
	std::shared_ptr<void> detector_gpu_ptr;
	std::deque<std::vector<bbox_t>> prev_bbox_vec_deque;
	const int cur_gpu_id;
public:
	float nms = .4;
	bool wait_stream;

	YOLODLL_API Detector(std::string cfg_filename, std::string weight_filename, int gpu_id = 0);
	YOLODLL_API ~Detector();

	YOLODLL_API std::vector<bbox_t> detect(std::string image_filename, float thresh = 0.2, bool use_mean = false);
	YOLODLL_API std::vector<bbox_t> detect(image_t img, float thresh = 0.2, bool use_mean = false);
	static YOLODLL_API image_t load_image(std::string image_filename);
	static YOLODLL_API void free_image(image_t m);
	YOLODLL_API int get_net_width() const;
	YOLODLL_API int get_net_height() const;

	YOLODLL_API std::vector<bbox_t> tracking_id(std::vector<bbox_t> cur_bbox_vec, bool const change_history = true, 
												int const frames_story = 10, int const max_dist = 150);

	std::vector<bbox_t> detect_resized(image_t img, int init_w, int init_h, float thresh = 0.2, bool use_mean = false)
	{
		if (img.data == NULL)
			throw std::runtime_error("Image is empty");
		auto detection_boxes = detect(img, thresh, use_mean);
		float wk = (float)init_w / img.w, hk = (float)init_h / img.h;
		for (auto &i : detection_boxes) i.x *= wk, i.w *= wk, i.y *= hk, i.h *= hk;
		return detection_boxes;
	}

	std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
	{
		if(mat.data == NULL)
			throw std::runtime_error("Image is empty");
		auto image_ptr = mat_to_image_resize(mat);
		return detect_resized(*image_ptr, mat.cols, mat.rows, thresh, use_mean);
	}

	std::shared_ptr<image_t> mat_to_image_resize(cv::Mat mat) const
	{
		if (mat.data == NULL) return std::shared_ptr<image_t>(NULL);
		cv::Mat det_mat;
		cv::resize(mat, det_mat, cv::Size(get_net_width(), get_net_height()));
		return mat_to_image(det_mat);
	}

	static std::shared_ptr<image_t> mat_to_image(cv::Mat img_src)
	{
		cv::Mat img;
		cv::cvtColor(img_src, img, cv::COLOR_RGB2BGR);
		std::shared_ptr<image_t> image_ptr(new image_t, [](image_t *img) { free_image(*img); delete img; });
		std::shared_ptr<IplImage> ipl_small = std::make_shared<IplImage>(img);
		*image_ptr = ipl_to_image(ipl_small.get());
		return image_ptr;
	}

private:

	static image_t ipl_to_image(IplImage* src)
	{
		unsigned char *data = (unsigned char *)src->imageData;
		int h = src->height;
		int w = src->width;
		int c = src->nChannels;
		int step = src->widthStep;
		image_t out = make_image_custom(w, h, c);
		int count = 0;

		for (int k = 0; k < c; ++k) {
			for (int i = 0; i < h; ++i) {
				int i_step = i*step;
				for (int j = 0; j < w; ++j) {
					out.data[count++] = data[i_step + j*c + k] / 255.;
				}
			}
		}

		return out;
	}

	static image_t make_empty_image(int w, int h, int c)
	{
		image_t out;
		out.data = 0;
		out.h = h;
		out.w = w;
		out.c = c;
		return out;
	}

	static image_t make_image_custom(int w, int h, int c)
	{
		image_t out = make_empty_image(w, h, c);
		out.data = (float *)calloc(h*w*c, sizeof(float));
		return out;
	}
};
// CPU Tracking Version

//#include <opencv2/optflow.hpp>
/*#include <opencv2/video/tracking.hpp>

class Tracker_optflow {
public:
	const int flow_error;


	Tracker_optflow(int win_size = 9, int max_level = 3, int iterations = 8000, int _flow_error = -1) :
		flow_error((_flow_error > 0)? _flow_error:(win_size*4))
	{
		sync_PyrLKOpticalFlow = cv::SparsePyrLKOpticalFlow::create();
		sync_PyrLKOpticalFlow->setWinSize(cv::Size(win_size, win_size));	// 9, 15, 21, 31
		sync_PyrLKOpticalFlow->setMaxLevel(max_level);		// +- 3 pt

	}

	// just to avoid extra allocations
	cv::Mat dst_grey;
	cv::Mat prev_pts_flow, cur_pts_flow;
	cv::Mat status, err;

	cv::Mat src_grey;	// used in both functions
	cv::Ptr<cv::SparsePyrLKOpticalFlow> sync_PyrLKOpticalFlow;

	std::vector<bbox_t> cur_bbox_vec;
	std::vector<bool> good_bbox_vec_flags;

	void update_cur_bbox_vec(std::vector<bbox_t> _cur_bbox_vec)
	{
		cur_bbox_vec = _cur_bbox_vec;
		good_bbox_vec_flags = std::vector<bool>(cur_bbox_vec.size(), true);
		cv::Mat prev_pts, cur_pts_flow;

		for (auto &i : cur_bbox_vec) {
			float x_center = (i.x + i.w / 2.0F);
			float y_center = (i.y + i.h / 2.0F);
			prev_pts.push_back(cv::Point2f(x_center, y_center));
		}

		if (prev_pts.rows == 0)
			prev_pts_flow = cv::Mat();
		else
			cv::transpose(prev_pts, prev_pts_flow);
	}


	void update_tracking_flow(cv::Mat new_src_mat, std::vector<bbox_t> _cur_bbox_vec)
	{
		if (new_src_mat.channels() == 3) {

			update_cur_bbox_vec(_cur_bbox_vec);

			cv::cvtColor(new_src_mat, src_grey, CV_BGR2GRAY, 1);
		}
	}


	std::vector<bbox_t> tracking_flow(cv::Mat new_dst_mat, bool check_error = true)
	{
		if (sync_PyrLKOpticalFlow.empty()) {
			std::cout << "sync_PyrLKOpticalFlow isn't initialized \n";
			return cur_bbox_vec;
		}

		cv::cvtColor(new_dst_mat, dst_grey, CV_BGR2GRAY, 1);

		if (src_grey.rows != dst_grey.rows || src_grey.cols != dst_grey.cols) {
			src_grey = dst_grey.clone();
			return cur_bbox_vec;
		}

		if (prev_pts_flow.cols < 1) {
			return cur_bbox_vec;
		}

		////sync_PyrLKOpticalFlow_gpu.sparse(src_grey_gpu, dst_grey_gpu, prev_pts_flow_gpu, cur_pts_flow_gpu, status_gpu, &err_gpu);	// OpenCV 2.4.x
		sync_PyrLKOpticalFlow->calc(src_grey, dst_grey, prev_pts_flow, cur_pts_flow, status, err);	// OpenCV 3.x

		dst_grey.copyTo(src_grey);

		std::vector<bbox_t> result_bbox_vec;

		if (err.rows == cur_bbox_vec.size() && status.rows == cur_bbox_vec.size())
		{
			for (size_t i = 0; i < cur_bbox_vec.size(); ++i)
			{
				cv::Point2f cur_key_pt = cur_pts_flow.at<cv::Point2f>(0, i);
				cv::Point2f prev_key_pt = prev_pts_flow.at<cv::Point2f>(0, i);

				float moved_x = cur_key_pt.x - prev_key_pt.x;
				float moved_y = cur_key_pt.y - prev_key_pt.y;

				if (abs(moved_x) < 100 && abs(moved_y) < 100 && good_bbox_vec_flags[i])
					if (err.at<float>(0, i) < flow_error && status.at<unsigned char>(0, i) != 0 &&
						((float)cur_bbox_vec[i].x + moved_x) > 0 && ((float)cur_bbox_vec[i].y + moved_y) > 0)
					{
						cur_bbox_vec[i].x += moved_x + 0.5;
						cur_bbox_vec[i].y += moved_y + 0.5;
						result_bbox_vec.push_back(cur_bbox_vec[i]);
					}
					else good_bbox_vec_flags[i] = false;
				else good_bbox_vec_flags[i] = false;

				//if(!check_error && !good_bbox_vec_flags[i]) result_bbox_vec.push_back(cur_bbox_vec[i]);
			}
		}

		prev_pts_flow = cur_pts_flow.clone();

		return result_bbox_vec;
	}

};
#else
*/
#endif	// __cplusplus
#endif // YOLODLL_API 
