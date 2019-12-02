#include "Initializer.h"

/*
Description:
This function is a contructor;
Input:
@ context& context: a reference to realsense context;
@ map<string, int>& asicSerialIndexMap: a reference to camera asic serial index map;
@ map<string, int>& serialIndexMap: a reference to camera serial index map;
@ map<int, colorizer>& colorizers: a reference to colorizers;
@ vector<pipeline>& pipelines: a reference to realsense pipelines;
*/
Initializer::Initializer(rs2::context& context, map<string, int>& asicSerialIndexMap, map<string, int>& serialIndexMap, map<int, colorizer>& colorizers, vector<pipeline>& pipelines) :
	context(context), asicSerialIndexMap(asicSerialIndexMap), serialIndexMap(serialIndexMap), colorizers(colorizers), pipelines(pipelines) {
}

/*
Description:
This function is used to initialize a map to save Camera Asic Serial number and camera index;
Input:
@ void parameter: void;
Output:
@ void returnValue:void;
*/
void Initializer::AsicSerialIndexInitialization() {
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
}

/*
Description:
This function is used to initialize a map to save Camera Serial number and camera index;
Input:
@ void parameter: void;
Output:
@ void returnValue:void;
*/
void Initializer::SerialIndexInitialization() {
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
}

/*
Description:
This function is used to print registered camera information including camera name, asic serail number, serial number, tag number, usb descriptor etc.
Input:
@ void parameter: void;
Output:
@ void returnValue: void;
*/
void Initializer::CameraInfoPrint() {
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
			return;
		}

		// Detect if USB Type Descriptor is 3.0 or higher version
		if (atof(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) < 3.0) {
			cout << "Please check USB Port, it may not be USB 3.0 or higher version" << endl;
			// return -1;
		}
	}
}

/*
Description:
This function is used to initialize realsense cameras;
Input:
@ void parameter: void;
Output:
@ void returnValue: void;
*/
void Initializer::CameraInitialization() {
	cout << endl << "--- Camera Initialization ---" << endl;
	for (auto&& device : context.query_devices()) {

		// initialization
		pipeline pipeline(context);
		config config;
		string serialNumber = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
		cout << "Initialize camera " << serialIndexMap.at(serialNumber) << endl;

		// set configuration
		config.enable_stream(rs2_stream::RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
		this->enable_stream_color = true;
		config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
		this->enable_stream_depth = true;
		//config.enable_all_streams();
		config.enable_device(serialNumber);

		// start the pipeline
		pipeline.start(config);
		cout << "Camera [" << serialNumber << "] streaming pipeline is starting..." << endl;
		pipelines.push_back(pipeline);
		colorizers[serialIndexMap[serialNumber]] = colorizer();
	}
}

bool Initializer::isEnableStreamColor() const {
	return enable_stream_color;
}

bool Initializer::isEnableStreamDepth() const {
	return enable_stream_depth;
}

bool Initializer::isEnableStreamInfrared() const {
	return enable_stream_infrared;
}
