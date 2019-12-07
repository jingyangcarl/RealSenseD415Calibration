#include <librealsense2/rs.hpp>
using namespace rs2;

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "Window.h"
#include "Initializer.h"

int main() try {

	// get current RealSense context
	rs2::context context;

	int width = 640;
	int height = 480;
	int frameRate = 30;
	int deviceSize = context.query_devices().size();
	window app(width * deviceSize, height * deviceSize, "Multi-Camera");

	// Init initializer
	map<string, int> asicSerialIndexMap;
	map<string, int> serialIndexMap;
	map<int, colorizer> colorizers;
	vector<pipeline> pipelines;
	Initializer initializer(context, asicSerialIndexMap, serialIndexMap, colorizers, pipelines);
	// RealSense D415 Asic Serial Number - Index Number
	initializer.AsicSerialIndexInitialization();

	// RealSense D415 Seerial Number - Index Number
	initializer.SerialIndexInitialization();

	// present the basic device information
	initializer.CameraInfoPrint();

	// start a streaming pipe per each connected device
	initializer.CameraInitialization();

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
				cout << "current camera: " << serialIndexMap[pipeline.get_active_profile().get_device().get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] << endl;
				cout << "colorFrame: " << (initializer.isEnableStreamColor() ? colorFrame.get_profile().unique_id() : -1) << endl;
				cout << "depthFrame: " << (initializer.isEnableStreamDepth() ? depthFrame.get_profile().unique_id() : -1) << endl;
				cout << "infraredFrame_1: " << (initializer.isEnableStreamInfrared() ? infraredFrame_1.get_profile().unique_id() : -1) << endl;
				cout << "infraredFrame_2: " << (initializer.isEnableStreamInfrared() ? infraredFrame_2.get_profile().unique_id() : -1) << endl;

				// save frame
				if (framesetCount % (frameRate*deviceSize) < deviceSize) {

					// save color image
					stringstream colorImagePath;
					colorImagePath << "./Generated/colorImage_cam" << serialIndexMap[pipeline.get_active_profile().get_device().get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] << "_" << framesetCount%2 << ".png";
					Mat colorImage(Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), Mat::AUTO_STEP);
					cv::cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);
					cv::imwrite(colorImagePath.str(), colorImage);
					cv::cvtColor(colorImage, colorImage, cv::COLOR_RGB2BGR);

					// print
					auto end = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					cout << colorImagePath.str() << " saved at " << ctime(&end);

					// save depth image
					stringstream depthImagePath;
					depthImagePath << "./Generated/depthImage_cam" << serialIndexMap[pipeline.get_active_profile().get_device().get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] << "_" << framesetCount % 2 << ".tiff";
					Mat depthImage(Size(depthFrame.get_width(), depthFrame.get_height()), CV_16UC1, (void*)depthFrame.get_data(), Mat::AUTO_STEP);
					cv::imwrite(depthImagePath.str(), depthImage);

					// print
					end = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					cout << depthImagePath.str() << " saved at " << ctime(&end);
					cout << "\x1b[A\x1b[A";
				}

				// console lines up
				cout << "\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A";
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
		//app.show(render_frames);

	}

}
catch (const rs2::error & e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}