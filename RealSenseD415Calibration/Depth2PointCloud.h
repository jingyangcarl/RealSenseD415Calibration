#ifndef _DEPTH2POINTCLOUD_
#define _DEPTH2POINTCLOUD_

#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <librealsense2/rs.hpp>

#include "MESH.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <WinSock2.h>
#include <ws2tcpip.h>

#define FLOAT_TYPE float

class Camera
{
public:
	rs2::pipeline pipe;
	std::string serial;
	double scale;	// depth scale
	rs2::config cfg;
	int id;

	sockaddr addr;
	int port;

	cv::Mat KK_depth;
	Eigen::Matrix4f Rt_depth_to_color;
	Eigen::Matrix4f Rt_curr_to_world;

	cv::Mat distCoeff;
};

class cvFrame
{
public:
	cv::Mat * color_frame;
	cv::Mat * depth_frame;

	cvFrame operator= (const cvFrame f)
	{
		this->color_frame = f.color_frame;
		this->depth_frame = f.depth_frame;
		return *this;
	}
};

// const cv::Mat KK = (cv::Mat_<float>(3, 3) << 365.5953, 0.0, 260.1922, 0.0, 365.5953, 209.5835, 0.0, 0.0, 1.0);
// const cv::Mat Rt = (cv::Mat_<float>(3, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat1i &X, cv::Mat1i &Y);

void meshgridTest(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y);

void meshgridG(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat &X, cv::Mat &Y);

void meshgridTestG(const cv::Range &xgv, const cv::Range &ygv, cv::Mat &X, cv::Mat &Y);

cv::Mat depthmap2pointmap(cv::Mat& depth, cv::Mat& KK_depth);

void pointmap2mesh(cv::Mat& pointmap, cv::Mat& mask, MESH<FLOAT_TYPE>& submesh);

void pointmap2mesh_counting(cv::Mat& pointmap, MESH<FLOAT_TYPE>& mesh, int& times, cv::Mat & vt, cv::Mat & f, Camera& cam);
//void pointmap2mesh_counting(cv::Mat& pointmap, cv::Mat& mask, MESH<FLOAT_TYPE>& mesh, int& times, cv::Mat & vt, cv::Mat & f);

void RemoveLongFace(MESH<FLOAT_TYPE>& mesh, float threshold, MESH<FLOAT_TYPE>& mesh_clean);

void selectmesh_vind(MESH<FLOAT_TYPE>& mesh, cv::Mat& vind, int width, int height, MESH<FLOAT_TYPE>& submesh);


#endif // !_DEPTH2POINTCLOUD_
