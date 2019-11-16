#include <librealsense2/rs.hpp>
using rs2::context;
using rs2::pipeline;
using rs2::config;
using rs2::depth_sensor;

#include <iostream>
#include <map>
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::pair;
using std::vector;

#include "Window.h"

int main() {

	window app(640, 480, "Multi-Camera");

	// RealSense D415 Asic Serial Number - Index Number
	map<string, int> serialIndexMap;
	serialIndexMap.insert(pair<string, int>("822213022021", 1));
	serialIndexMap.insert(pair<string, int>("822213021828", 2));
	serialIndexMap.insert(pair<string, int>("822213022078", 3));
	serialIndexMap.insert(pair<string, int>("825513022964", 4));
	serialIndexMap.insert(pair<string, int>("821413023071", 5));
	serialIndexMap.insert(pair<string, int>("822213020008", 6));
	serialIndexMap.insert(pair<string, int>("821413025408", 7));
	serialIndexMap.insert(pair<string, int>("821413025392", 8));
	serialIndexMap.insert(pair<string, int>("821413023843", 9));
	serialIndexMap.insert(pair<string, int>("822213021799", 10));
	serialIndexMap.insert(pair<string, int>("822213021721", 11));
	serialIndexMap.insert(pair<string, int>("825513022115", 12));
	serialIndexMap.insert(pair<string, int>("822213022092", 13));
	serialIndexMap.insert(pair<string, int>("821413021123", 14));


	// get current RealSense context
	rs2::context context;

	// present the basic device information
	cout << "--- " << context.query_devices().size() << " Camera Detected ---" << endl;
	for (auto&& device : context.query_devices()) {
		cout << "Camera Info Name: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << endl;
		cout << "Camera Info Asic Serial Number: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << endl;
		cout << "Camera Info Tag Number: " << serialIndexMap.at(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)) << endl;
		cout << "Camera Info USB Type Descriptor: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << endl;

		// Detect if devices are all Intel RealSense D415
		if (strcmp(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME), "Intel RealSense D415") != 0) {
			cout << "Please check Camera Type, it may not be Intel RealSense D415" << endl;
			return -1;
		}

		// Detect if USB Type Descriptor is 3.2
		if (strcmp(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR), "3.2") != 0) {
			cout << "Please check USB Port, it may not be USB 3.2" << endl;
			return -1;
		}
	}

	// start a streaming pipe per each connected device
	map<int, float> indexDepthScaleMap;
	vector<pipeline> pipelines;
	for (auto&& device : context.query_devices()) {

		// initialization
		pipeline pipeline(context);
		config config;
		string asicSerialNumber = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER);

		// get depth scale
		float depthScale = device.query_sensors().front().as<depth_sensor>().get_depth_scale();
		int index = serialIndexMap.at(asicSerialNumber);
		indexDepthScaleMap.insert(pair<int, float>(index, depthScale));

		// set configuration
		int width = 640;
		int height = 480;
		int frameRate = 30;
		//config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, frameRate);
		//config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, frameRate);
		config.enable_device(asicSerialNumber);

		// start the pipeline
		pipeline.start(config);
		pipelines.push_back(pipeline);
		cout << "Start pipeline of camera " << serialIndexMap.at(asicSerialNumber) << endl;

	}

	

}