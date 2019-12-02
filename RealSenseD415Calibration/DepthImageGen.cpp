// DepthImageGen.cpp : This file contains the 'main' function. Program execution begins and ends there.
/*
 Usage: This code can capture images automatically from at most 4 cameras.
 First, prepare a scene (for example, a calibration board), run the code.
 The program will halt for 10 seconds for you to do extra preparation (for example go and hold the board in front of the cameras).
 (In a single group, each camera will produce one image, one depth data, and one point cloud from the same scene.)
 Then the program will start capturing at a frequency of 3-4 seconds per group.
 During the 3-4 seconds, you can change position for the next capturing.
 Among all the global variables, there is one called 'num_of_pics'. You can modify it to change the number of groups you want to capture.
 
 The output files are named like this:
 RGB png file: 'cam[0-3]_[0-num_of_pics].png'. The first number means camera, and the second means group.
 Depth csv file: 'Depth_Image[0-3]_[0-num_of_pics].csv'
 Point cloud ply file: 'pointcloud_[0-3]_[0-num_of_pics].ply'
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <direct.h>
#include <iostream>
#include <string>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <C:/Program Files (x86)/Intel RealSense SDK 2.0/include/librealsense2/rs_advanced_mode.hpp>
#include <iostream>
#include <fstream>
#include <map>
#include <thread>
#include <string>
#include <vector>
#include <Windows.h>

using namespace rs2;
using namespace std;

const int num_of_pics = 30;
const int width = 1280;
const int height = 720;
const char* title = "D415 camera window";

void writeCSV(std::string filename, cv::Mat m)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}

int main() try
{
	context ctx;
	//map<string, pipeline> pipes;
	vector<pipeline> pipes;
	rs2::colorizer color_map;
	float scale[4];

	rs2::align color_align(RS2_STREAM_COLOR);

	int cnt_cam = 0;
	for (auto dev : ctx.query_devices()) // For each device do :
	{
		if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D415") == 0) // Check for compatibility
		{
			pipeline p;
			config cfg;
			string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			string json_file_name = "test.json";
			cout << "Configuring camera : " << serial << endl;

			scale[cnt_cam] = dev.query_sensors().front().as<depth_sensor>().get_depth_scale();

			// Add desired stream to configuration
			cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
			cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
			//cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_ANY, 30);
			cfg.enable_device(serial);

			// Start the pipeline
			p.start(cfg);
			cout << "Starting pipeline for current camera " << serial << endl;
			pipes.push_back(p);

			cnt_cam++;
		}
	}

	for (int m = 0; m < 9000000; m++);
	Sleep(10000);

	int k = 0;
	while (k < num_of_pics)
	{
		std::cout << "Ready to capture the " << k << "-th image\n";
		std::vector<rs2::frameset> new_frames;
		for (auto &it : pipes)
		{
			rs2::frameset frames;
			frames = (it).wait_for_frames();
			rs2::frameset fs = color_align.process(frames);
			new_frames.push_back(fs);
		}
		int i = 0;
		for (auto &fs : new_frames)
		{
			auto color = fs.get_color_frame();
			auto depth = fs.get_depth_frame();
			//pc.map_to(color);
			std::stringstream color_file, ir_file, depth_file, pc_file;
			//color_file << "Color_Image_" << i << "_" << k << ".png";
			color_file << "cam" << i << "_" << k << ".png";
			//color_file << "Infrared_Image_" << i << "_" << k << ".png";
			depth_file << "Depth_Image_" << i << "_" << k << ".csv";
			pc_file << "pointcloud_" << i << "_" << k << ".ply";


			//cv::Mat image(cv::Size(depth.get_width(), depth.get_height()), CV_8UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
			cv::Mat image1(color.get_height(), color.get_width(), CV_8UC3, (void *)(color.get_data()), cv::Mat::AUTO_STEP);
			cv::cvtColor(image1, image1, CV_BGR2RGB);
			cv::imwrite(color_file.str(), image1);
			

			float minClip = 0.4, maxClip = 4;
			const uint16_t * depth_ptr = (uint16_t *)(depth.get_data());
			cv::Mat image2(depth.get_height(), depth.get_width(), CV_16UC1, (void *)(depth.get_data()), cv::Mat::AUTO_STEP);

			for (int m = 0; m < depth.get_height(); m++)
			{
				auto depth_pixel_index = m * depth.get_width();
				for (int j = 0; j < depth.get_width(); j++)
				{
					auto dist = depth_ptr[depth_pixel_index + j] * scale[i];
					image2.at<uint16_t>(m, j) = (dist <= maxClip && dist >= minClip) ? (image2.at<uint16_t>(m, j)) : 0.0;
				}
			}

			//cv::imwrite(depth_file.str(), image2);
			writeCSV(depth_file.str(), image2);

			rs2::pointcloud pc;
			rs2::points points;

			pc.map_to(fs.get_color_frame());
			// Generate the pointcloud and texture mappings
			points = pc.calculate(depth);
			points.export_to_ply(pc_file.str(), fs.get_color_frame());

			cv::cvtColor(image1, image1, CV_BGR2RGB);

			cv::imshow("out", image1);
			cv::waitKey(1);
			Sleep(1000);

			i++;
		}
		k++;
		Sleep(2000);

	}

	return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}

catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}