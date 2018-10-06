#include "detector-wrapper.hpp"

#include "../network.h"

extern "C" {
#include "../detection_layer.h"
#include "../region_layer.h"
#include "../cost_layer.h"
#include "../utils.h"
#include "../parser.h"
#include "../box.h"
#include "../image.h"
#include "../demo.h"
#include "../option_list.h"
#include "../stb_image.h"
}

#include <vector>
#include <iostream>
#include <algorithm>

#define FRAMES 3

void check_cuda(cudaError_t status) {
	if (status != cudaSuccess) {
		const char *s = cudaGetErrorString(status);
		printf("CUDA Error Prev: %s\n", s);
	}
}

struct detector_gpu_t {
	network net;
	image images[FRAMES];
	float *avg;
	float *predictions[FRAMES];
	int demo_index;
	unsigned int *track_id;
};

YOLODLL_API Detector::Detector(std::string cfg_filename, std::string weight_filename, int gpu_id) : cur_gpu_id(gpu_id)
{
	wait_stream = 0;
	int old_gpu_index;
	check_cuda( cudaGetDevice(&old_gpu_index) );

	detector_gpu_ptr = std::make_shared<detector_gpu_t>();
	detector_gpu_t &detector_gpu = *static_cast<detector_gpu_t *>(detector_gpu_ptr.get());

	//check_cuda( cudaSetDevice(cur_gpu_id) );
	cuda_set_device(cur_gpu_id);
	printf(" Used GPU %d \n", cur_gpu_id);
	network &net = detector_gpu.net;
	net.gpu_index = cur_gpu_id;
	//gpu_index = i;
	
	char *cfgfile = const_cast<char *>(cfg_filename.data());
	char *weightfile = const_cast<char *>(weight_filename.data());

	net = parse_network_cfg_custom(cfgfile, 1);
	if (weightfile) {
		load_weights(&net, weightfile);
	}
	set_batch_network(&net, 1);
	net.gpu_index = cur_gpu_id;
	fuse_conv_batchnorm(net);

	layer l = net.layers[net.n - 1];
	int j;

	detector_gpu.avg = (float *)calloc(l.outputs, sizeof(float));
	for (j = 0; j < FRAMES; ++j) detector_gpu.predictions[j] = (float *)calloc(l.outputs, sizeof(float));
	for (j = 0; j < FRAMES; ++j) detector_gpu.images[j] = make_image(1, 1, 3);

	detector_gpu.track_id = (unsigned int *)calloc(l.classes, sizeof(unsigned int));
	for (j = 0; j < l.classes; ++j) detector_gpu.track_id[j] = 1;

	check_cuda( cudaSetDevice(old_gpu_index) );
}


YOLODLL_API Detector::~Detector() 
{
	detector_gpu_t &detector_gpu = *static_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	layer l = detector_gpu.net.layers[detector_gpu.net.n - 1];

	free(detector_gpu.track_id);

	free(detector_gpu.avg);
	for (int j = 0; j < FRAMES; ++j) free(detector_gpu.predictions[j]);
	for (int j = 0; j < FRAMES; ++j) if(detector_gpu.images[j].data) free(detector_gpu.images[j].data);

	int old_gpu_index;
	cudaGetDevice(&old_gpu_index);
	cuda_set_device(detector_gpu.net.gpu_index);

	free_network(detector_gpu.net);

	cudaSetDevice(old_gpu_index);
}

YOLODLL_API int Detector::get_net_width() const {
	detector_gpu_t &detector_gpu = *static_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	return detector_gpu.net.w;
}
YOLODLL_API int Detector::get_net_height() const {
	detector_gpu_t &detector_gpu = *static_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	return detector_gpu.net.h;
}

YOLODLL_API std::vector<bbox_t> Detector::detect(std::string image_filename, float thresh, bool use_mean)
{
	std::shared_ptr<image_t> image_ptr(new image_t, [](image_t *img) { if (img->data) free(img->data); delete img; });
	*image_ptr = load_image(image_filename);
	return detect(*image_ptr, thresh, use_mean);
}

static image load_image_stb(char *filename, int channels)
{
	int w, h, c;
	unsigned char *data = stbi_load(filename, &w, &h, &c, channels);
	if (!data) 
		throw std::runtime_error("file not found");
	if (channels) c = channels;
	int i, j, k;
	image im = make_image(w, h, c);
	for (k = 0; k < c; ++k) {
		for (j = 0; j < h; ++j) {
			for (i = 0; i < w; ++i) {
				int dst_index = i + w*j + w*h*k;
				int src_index = k + c*i + c*w*j;
				im.data[dst_index] = (float)data[src_index] / 255.;
			}
		}
	}
	free(data);
	return im;
}

YOLODLL_API image_t Detector::load_image(std::string image_filename)
{
	char *input = const_cast<char *>(image_filename.data());
	image im = load_image_stb(input, 3);

	image_t img;
	img.c = im.c;
	img.data = im.data;
	img.h = im.h;
	img.w = im.w;

	return img;
}

YOLODLL_API void Detector::free_image(image_t m)
{
	if (m.data) {
		free(m.data);
	}
}

YOLODLL_API std::vector<bbox_t> Detector::detect(image_t img, float thresh, bool use_mean)
{	
	detector_gpu_t &detector_gpu = *static_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	network &net = detector_gpu.net;
	int old_gpu_index;
	cudaGetDevice(&old_gpu_index);
	if(cur_gpu_id != old_gpu_index)
		cudaSetDevice(net.gpu_index);

	net.wait_stream = wait_stream;	// 1 - wait CUDA-stream, 0 - not to wait
	//std::cout << "net.gpu_index = " << net.gpu_index << std::endl;

	//float nms = .4;

	image im;
	im.c = img.c;
	im.data = img.data;
	im.h = img.h;
	im.w = img.w;

	image sized;
	
	if (net.w == im.w && net.h == im.h) {
		sized = make_image(im.w, im.h, im.c);
		memcpy(sized.data, im.data, im.w*im.h*im.c * sizeof(float));
	}
	else
		sized = resize_image(im, net.w, net.h);

	layer l = net.layers[net.n - 1];

	float *X = sized.data;

	float *prediction = network_predict(net, X);

	if (use_mean) {
		memcpy(detector_gpu.predictions[detector_gpu.demo_index], prediction, l.outputs * sizeof(float));
		mean_arrays(detector_gpu.predictions, FRAMES, l.outputs, detector_gpu.avg);
		l.output = detector_gpu.avg;
		detector_gpu.demo_index = (detector_gpu.demo_index + 1) % FRAMES;
	}
	//get_region_boxes(l, 1, 1, thresh, detector_gpu.probs, detector_gpu.boxes, 0, 0);
	//if (nms) do_nms_sort(detector_gpu.boxes, detector_gpu.probs, l.w*l.h*l.n, l.classes, nms);

	int nboxes = 0;
	int letterbox = 0;
	float hier_thresh = 0.5;
	detection *dets = get_network_boxes(&net, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes, letterbox);
	if (nms) do_nms_sort(dets, nboxes, l.classes, nms);

	std::vector<bbox_t> bbox_vec;

	for (size_t i = 0; i < nboxes; ++i) {
		box b = dets[i].bbox;
		int const obj_id = max_index(dets[i].prob, l.classes);
		float const prob = dets[i].prob[obj_id];
		
		if (prob > thresh) 
		{
			bbox_t bbox;
			bbox.x = std::max((double)0, (b.x - b.w / 2.)*im.w);
			bbox.y = std::max((double)0, (b.y - b.h / 2.)*im.h);
			bbox.w = b.w*im.w;
			bbox.h = b.h*im.h;
			bbox.obj_id = obj_id;
			bbox.prob = prob;
			bbox.track_id = 0;

			bbox_vec.push_back(bbox);
		}
	}

	free_detections(dets, nboxes);
	if(sized.data)
		free(sized.data);

	if (cur_gpu_id != old_gpu_index)
		cudaSetDevice(old_gpu_index);

	return bbox_vec;
}