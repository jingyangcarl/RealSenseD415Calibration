#include <librealsense2/rs.hpp>
using rs2::context;
using rs2::pipeline;
using rs2::config;
using rs2::depth_sensor;
using rs2::frame;
using rs2::frameset;
using rs2::colorizer;
using rs2::video_frame;
using rs2::depth_frame;

#include <iostream>
using std::cout;
using std::endl;
#include <map>
using std::map;
#include <string>
using std::string;
using std::stringstream;
#include <vector>
using std::pair;
using std::vector;

#include <opencv2/opencv.hpp>
using cv::Mat;
using cv::Size;

#include "Window.h"

int main() {

	// get current RealSense context
	rs2::context context;
	int width = 640;
	int height = 480;
	int frameRate = 30;
	int deviceSize = context.query_devices().size();
	window app(width * deviceSize, height * deviceSize, "Multi-Camera");

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
	asicSerialIndexMap.insert(pair<string, int>("821413020551", 15));

	// RealSense D415 Seerial Number - Index Number
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
	serialIndexMap.insert(pair<string, int>("821212061289", 15));

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

		// Detect if USB Type Descriptor is 3.0 or higher version
		if (atof(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) < 3.0) {
			cout << "Please check USB Port, it may not be USB 3.0 or higher version" << endl;
			// return -1;
		}
	}

	// start a streaming pipe per each connected device
	map<int, float> indexDepthScaleMap;
	map<int, colorizer> colorizers;
	vector<pipeline> pipelines;
	cout << endl << "--- Camera Initialization ---" << endl;
	for (auto&& device : context.query_devices()) {

		// initialization
		pipeline pipeline(context);
		config config;
		string serialNumber = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
		cout << "Initialize camera " << serialIndexMap.at(serialNumber) << endl;

		// get depth scale
		float depthScale = device.query_sensors().front().as<depth_sensor>().get_depth_scale();
		int index = serialIndexMap.at(serialNumber);
		indexDepthScaleMap.insert(pair<int, float>(index, depthScale));

		// set configuration
		//config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, frameRate);
		//config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, frameRate);
		//config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, frameRate);
		//config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, frameRate);
		config.enable_all_streams();
		config.enable_device(serialNumber);

		// start the pipeline
		pipeline.start(config);
		cout << "Camera [" << serialNumber << "] streaming pipeline is starting..." << endl;
		pipelines.push_back(pipeline);
		colorizers[serialIndexMap[serialNumber]] = colorizer();
	}

	// Main app loop
	map<int, frame> render_frames;
	int framesetCount(0);
	cout << endl << "--- Running the App ---" << endl;
	while (app) {

		// collect the new_frames from all the connected devices
		vector<frame> new_frames;
		frameset frameset;
		for (auto&& pipeline : pipelines) {
			if (pipeline.poll_for_frames(&frameset)) {

				// push frames
				for (const frame& frame : frameset) {
					new_frames.push_back(frame);
				}

				// get frames
				auto colorFrame = frameset.get_color_frame();
				auto depthFrame = frameset.get_depth_frame();
				auto infraredFrame_1 = frameset.get_infrared_frame(1);
				auto infraredFrame_2 = frameset.get_infrared_frame(2);

				// print frames
				cout << "colorFrame: " << colorFrame.get_profile().unique_id() << endl;
				cout << "depthFrame: " << depthFrame.get_profile().unique_id() << endl;
				cout << "infraredFrame_1: " << infraredFrame_1.get_profile().unique_id() << endl;
				cout << "infraredFrame_2: " << infraredFrame_2.get_profile().unique_id() << endl;
				//cout << "\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A";

				// save frame
				if (framesetCount % (frameRate*deviceSize) < deviceSize) {
					stringstream colorImagePath;
					colorImagePath << "./Generated/colorImage_" << serialIndexMap[pipeline.get_active_profile().get_device().get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] << ".png";
					Mat colorImage(Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), Mat::AUTO_STEP);
					cv::cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);
					cv::imwrite(colorImagePath.str(), colorImage);
					cv::cvtColor(colorImage, colorImage, cv::COLOR_RGB2BGR);

					stringstream depthImagePath;
					depthImagePath << "./Generated/depthImage_" << serialIndexMap[pipeline.get_active_profile().get_device().get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] << ".tiff";
					Mat depthImage(Size(depthFrame.get_width(), depthFrame.get_height()), CV_16UC1, (void*)depthFrame.get_data(), Mat::AUTO_STEP);
					cv::imwrite(depthImagePath.str(), depthImage);
				}

				framesetCount++;
			}
		}

		// convert the newly-arrived new_frames to render-friendly format
		for (const auto& frame : new_frames) {
			// get the serical number of the current frame's device
			auto serialNumber = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			// apply the colorizer of the matching device and store the colorzied frame
			render_frames[frame.get_profile().unique_id()] = colorizers[serialIndexMap[serialNumber]].process(frame);
		}

		// present all the collected new_frames with Opengl mosiaic
		app.show(render_frames);

	}

}