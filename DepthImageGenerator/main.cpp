#include <librealsense2/rs.hpp>
using rs2::context;
using rs2::pipeline;
using rs2::config;
using rs2::depth_sensor;
using rs2::frame;
using rs2::frameset;
using rs2::colorizer;

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

	window app(1280, 960, "Multi-Camera");

	// RealSense D415 Asic Serial Number - Index Number
	map<string, int> asicSerialIndexMap;
	asicSerialIndexMap.insert(pair<string, int>("822213022021", 1));
	asicSerialIndexMap.insert(pair<string, int>("822213021828", 2));
	asicSerialIndexMap.insert(pair<string, int>("822213022078", 3));
	asicSerialIndexMap.insert(pair<string, int>("825513022964", 4));
	asicSerialIndexMap.insert(pair<string, int>("821413023071", 5));
	asicSerialIndexMap.insert(pair<string, int>("822213020008", 6));
	asicSerialIndexMap.insert(pair<string, int>("821413025408", 7));
	asicSerialIndexMap.insert(pair<string, int>("821413025392", 8));
	asicSerialIndexMap.insert(pair<string, int>("821413023843", 9));
	asicSerialIndexMap.insert(pair<string, int>("822213021799", 10));
	asicSerialIndexMap.insert(pair<string, int>("822213021721", 11));
	asicSerialIndexMap.insert(pair<string, int>("825513022115", 12));
	asicSerialIndexMap.insert(pair<string, int>("822213022092", 13));
	asicSerialIndexMap.insert(pair<string, int>("821413021123", 14));

	map<string, int> serialIndexMap;
	serialIndexMap.insert(pair<string, int>("821212061662", 1));
	serialIndexMap.insert(pair<string, int>("821312061467", 2));
	serialIndexMap.insert(pair<string, int>("821212061641", 3));
	serialIndexMap.insert(pair<string, int>("822512061371", 4));
	serialIndexMap.insert(pair<string, int>("821212061237", 5));
	serialIndexMap.insert(pair<string, int>("821212061149", 6));
	serialIndexMap.insert(pair<string, int>("821312062319", 7));
	serialIndexMap.insert(pair<string, int>("821212061847", 8));
	serialIndexMap.insert(pair<string, int>("821312061266", 9));
	serialIndexMap.insert(pair<string, int>("821212060773", 10));
	serialIndexMap.insert(pair<string, int>("821212061889", 11));
	serialIndexMap.insert(pair<string, int>("822512060704", 12));
	serialIndexMap.insert(pair<string, int>("821212061615", 13));
	serialIndexMap.insert(pair<string, int>("821212061280", 14));

	// get current RealSense context
	rs2::context context;

	// present the basic device information
	cout << "--- " << context.query_devices().size() << " Camera Detected ---" << endl;
	for (auto&& device : context.query_devices()) {
		cout << "Camera Info Name: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << endl;
		cout << "Camera Info Asic Serial Number: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << endl;
		cout << "Camera Info Serial Number: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER) << endl;
		cout << "Camera Info Tag Number: " << serialIndexMap[device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] << endl;
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
	map<int, colorizer> colorizers;
	vector<pipeline> pipelines;
	for (auto&& device : context.query_devices()) {


		// initialization
		pipeline pipeline(context);
		config config;
		string serialNumber = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
		cout << "Initialize camera " << serialIndexMap.at(serialNumber) << " [" << serialNumber << "]" << endl;

		// get depth scale
		float depthScale = device.query_sensors().front().as<depth_sensor>().get_depth_scale();
		int index = serialIndexMap.at(serialNumber);
		indexDepthScaleMap.insert(pair<int, float>(index, depthScale));

		// set configuration
		int width = 640;
		int height = 480;
		int frameRate = 30;
		//config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, frameRate);
		//config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, frameRate);
		config.enable_device(serialNumber);

		// start the pipeline
		pipeline.start(config);
		pipelines.push_back(pipeline);
		colorizers[serialIndexMap[serialNumber]] = colorizer();
	}

	// Main app loop
	map<int, frame> render_frames;
	while (app) {

		// collect the new frames from all the connected devices
		vector<frame> frames;
		for (auto&& pipeline : pipelines) {
			frameset frameset;
			if (pipeline.poll_for_frames(&frameset)) {
				for (const frame& frame : frameset) {
					frames.push_back(frame);
				}
			}
		}

		// convert the newly-arrived frames to render-friendly format
		for (const auto& frame : frames) {
			// get the serical number of the current frame's device
			auto asicSerialNumber = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			// apply the colorizer of the matching device and store the colorzied frame
			render_frames[frame.get_profile().unique_id()] = colorizers[serialIndexMap[asicSerialNumber]].process(frame);
		}

		// present all the collected frames with Opengl mosiaic
		app.show(render_frames);

	}

}