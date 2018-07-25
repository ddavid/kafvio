#include "detector-wrapper.hpp"

#include "network.h"

extern "C" {
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include "option_list.h"
#include "stb_image.h"
}

#include <vector>
#include <iostream>
#include <algorithm>

YOLODLL_API std::vector<bbox_t> Detector::tracking_id(std::vector<bbox_t> cur_bbox_vec, bool const change_history, int const frames_story, int const max_dist)
{
  detector_gpu_t &det_gpu = *static_cast<detector_gpu_t *>(detector_gpu_ptr.get());

  bool prev_track_id_present = false;
  for (auto &i : prev_bbox_vec_deque)
    if (i.size() > 0) prev_track_id_present = true;

  if (!prev_track_id_present) {
    for (size_t i = 0; i < cur_bbox_vec.size(); ++i)
      cur_bbox_vec[i].track_id = det_gpu.track_id[cur_bbox_vec[i].obj_id]++;
    prev_bbox_vec_deque.push_front(cur_bbox_vec);
    if (prev_bbox_vec_deque.size() > frames_story) prev_bbox_vec_deque.pop_back();
    return cur_bbox_vec;
  }

  std::vector<unsigned int> dist_vec(cur_bbox_vec.size(), std::numeric_limits<unsigned int>::max());

  for (auto &prev_bbox_vec : prev_bbox_vec_deque) {
    for (auto &i : prev_bbox_vec) {
      int cur_index = -1;
      for (size_t m = 0; m < cur_bbox_vec.size(); ++m) {
        bbox_t const& k = cur_bbox_vec[m];
        if (i.obj_id == k.obj_id) {
          float center_x_diff = (float)(i.x + i.w/2) - (float)(k.x + k.w/2);
          float center_y_diff = (float)(i.y + i.h/2) - (float)(k.y + k.h/2);
          unsigned int cur_dist = sqrt(center_x_diff*center_x_diff + center_y_diff*center_y_diff);
          if (cur_dist < max_dist && (k.track_id == 0 || dist_vec[m] > cur_dist)) {
            dist_vec[m] = cur_dist;
            cur_index = m;
          }
        }
      }

      bool track_id_absent = !std::any_of(cur_bbox_vec.begin(), cur_bbox_vec.end(), 
        [&i](bbox_t const& b) { return b.track_id == i.track_id && b.obj_id == i.obj_id; });

      if (cur_index >= 0 && track_id_absent){
        cur_bbox_vec[cur_index].track_id = i.track_id;
        cur_bbox_vec[cur_index].w = (cur_bbox_vec[cur_index].w + i.w) / 2;
        cur_bbox_vec[cur_index].h = (cur_bbox_vec[cur_index].h + i.h) / 2;
      }
    }
  }

  for (size_t i = 0; i < cur_bbox_vec.size(); ++i)
    if (cur_bbox_vec[i].track_id == 0)
      cur_bbox_vec[i].track_id = det_gpu.track_id[cur_bbox_vec[i].obj_id]++;

  if (change_history) {
    prev_bbox_vec_deque.push_front(cur_bbox_vec);
    if (prev_bbox_vec_deque.size() > frames_story) prev_bbox_vec_deque.pop_back();
  }

  return cur_bbox_vec;
}
