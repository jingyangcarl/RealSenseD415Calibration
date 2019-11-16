/*
 For this program, first put the color and depth files with correct file names in the folder. Then run the program.
 From output in the command line, select the matrix with lowest cost as the final result. (I didn't write this part...)
 */

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Depth2PointCloud.h"
#include <opencv2/ml.hpp>

#include "sophus/se3.hpp"

#include <direct.h>
#include <windows.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define MAXV(a,b)  ((a) < (b) ? (b) : (a))

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VecVector3f;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector3f>> VecVector2f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

// Input is a cv::Mat, output is a vector<Point2f>.
std::vector<cv::Point2f> charuco2vector(const cv::Mat& ch)
{
	int rows = ch.rows;
	std::vector<cv::Point2f> res;

	for (int i = 0; i < rows; i++)
	{
		res.push_back(cv::Point2f(ch.at<float>(i, 0), ch.at<float>(i, 1)));
	}

	return res;
}

// For debug
void print(std::vector<std::vector<cv::Point2f>>& corners)
{
	std::cout << "x: " << corners.size() << " y: " << corners[0].size() << "\n";
	for (int i = 0; i < corners.size(); i++)
	{
		for (int j = 0; j < corners[0].size(); j++)
		{
			std::cout << corners[i][j].x << " " << corners[i][j].y << "\n";
		}
	}
	std::cout << "\n\n";
	return;
}

// For debug
void writeCSV(std::string filename, cv::Mat m)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}

cv::Mat extractDepthFile(const std::string& depthName);

/*
 stereo calibration using charuco-board
 */
static void StereoCalibCharuco(
	const std::vector<std::string>& imageList,	// input images
	int squaresX,								// x-axis suqare num
	int squaresY,								// y-axis square num
	float squareLength,							// square length
	float markerLength,							// charuco marker length
	int dictionaryId = 10						// charuco pattern
)
{
	// number of images must be even to be paired
	if (imageList.size() % 2 != 0)
	{
		std::cout << "Error: the image list contains odd number of elements\n";
		return;
	}

	bool refindStrategy = true;	// enhance the result of finding corners

	// 2 - D corner points in all imagea. The[2] means image pair
	// vector<vector<vector<point2f>>> allCorners[2]
	//   |      |      |       |                  |
	//   v      v      v       v                  v
	// image markers 4corners coordinate         image group
	std::vector<std::vector<std::vector<cv::Point2f>>> allCorners[2];
	// 2-D corner points in all images. The [2] means image pair
	// image, markers, coordinate. and image group. Same for the following variables.
	std::vector<std::vector<cv::Point2f>> imagePoints[2];
	// 3-D corner points in all images. There is no [2] because in an image pair, the 3-D coordinate is the same
	std::vector<std::vector<cv::Point3f>> objectPoints;
	// corner point ID in one image
	std::vector<std::vector<int>> allIds;
	// Image path sequence
	std::vector<std::string> allImgs;
	// Image ID sequence
	std::vector<int> allImgIds;
	cv::Size imgSize;

	// OpenCV function, defined the charuco pattern (in OpenCV we call it dictionary) used on the charuco-board
	cv::Ptr<cv::aruco::Dictionary> dictionary =
		cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	// create charuco board object according to width, height, square length and marker length
	cv::Ptr<cv::aruco::CharucoBoard> charucoboard =
		cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
	cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

	// Detector parameters. https://docs.opencv.org/master/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

	// i: The i-th image group
	// j: Count the number of accepted image groups. j <= i
	// k: 0 <= k < 2, each image pair
	// nimages: number of image groups
	int i, j, k, nimages = (int)imageList.size() / 2;

	allCorners[0].resize(nimages);
	allCorners[1].resize(nimages);

	// Detect corner points on each pair of image. Generate 2-D data
	for (i = j = 0; i < nimages; i++)	// traverse through pairs
	{
		std::cout << i << "round\n";
		std::vector<int> imgIds[2];	// marker IDs in the pair
		std::vector<cv::Point2f> charucoCorners[2];
		for (k = 0; k < 2; k++)	// image 0, image 1
		{
			// Get image
			cv::Mat imgCopy;
			const std::string& filename = imageList[i * 2 + k];
			cv::Mat img = cv::imread(filename, 0);

			if (img.empty()) break;
			if (imgSize == cv::Size()) imgSize = img.size();
			else if (img.size() != imgSize)
			{
				std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;

			std::vector<int>& ids = imgIds[k];	// marker IDs in image k
			std::vector<std::vector<cv::Point2f>>& corners = allCorners[k][j];
			std::vector<std::vector<cv::Point2f>> rejected;	// Rejected images

			// detect markers
			cv::aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);

			// refine strategy to detect more markers
			if (refindStrategy) cv::aruco::refineDetectedMarkers(img, board, corners, ids, rejected);

			// interpolate charuco corners
			cv::Mat currentCharucoCorners, currentCharucoIds;
			// There should be 17 markers in one image (according to the charuco pattern). After refine strategy detection, if the number is still less than 17, reject this pair and quit
			if (ids.size() != 17)
			{
				k = 0;
				break;
			}
			if (ids.size() > 0)
			{
				cv::aruco::interpolateCornersCharuco(corners, ids, img, charucoboard, currentCharucoCorners, currentCharucoIds);
			}
			charucoCorners[k] = charuco2vector(currentCharucoCorners);
			print(corners);

			// draw results
			img.copyTo(imgCopy);
			if (ids.size() > 0) cv::aruco::drawDetectedMarkers(imgCopy, corners);

			if (currentCharucoCorners.total() > 0)
				cv::aruco::drawDetectedCornersCharuco(imgCopy, currentCharucoCorners, currentCharucoIds);
			/*
			cv::putText(imgCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

			cv::imshow("out", imgCopy);
			cv::waitKey(1);
			//system("pause");
			*/
			std::cout << "Frame captured" << std::endl;
			imgSize = img.size();
			//Sleep(500);
		}
		if (k == 2)	// Pair accepted. Store the pair for calibration and bundle-adjustment
		{
			allImgs.push_back(imageList[i * 2]);
			allImgs.push_back(imageList[i * 2 + 1]);
			allImgIds.push_back(i);
			allIds.push_back(imgIds[0]);
			allIds.push_back(imgIds[1]);
			imagePoints[0].push_back(charucoCorners[0]);
			imagePoints[1].push_back(charucoCorners[1]);
			j++;
		}
		
	}
	std::cout << j << "pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		std::cout << "Error: too little pairs to run the calibration\n";
		return;
	}

	// In one pair, assume the z-values for all corner points are the same. (Regard the board coord as the world coord)
	objectPoints.resize(nimages);
	// Generate 3-D data for each corner point
	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < squaresY - 1; j++)
		{
			for (k = 0; k < squaresX - 1; k++)
			{
				objectPoints[i].push_back(cv::Point3f(j * squareLength, k * squareLength, 10));	// Since z-values are the same, you can change 10 here, to whatever non-negative number you want. The X and Y value come from the physical value of the board
			}
		}
	}

	std::cout << imagePoints[0][0].size() << "\n";
	std::cout << imagePoints[1][0].size() << "\n";

	std::cout << "Running stereo calibration ...\n";

	////////////////////////////////////////////////////////// Calibrartion //////////////////////////////////////////////////////////
	cv::Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imgSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imgSize, 0);
	cv::Mat R, T, E, F;

	std::cout << "initCameraMatrix2D finised ...\n";

	float rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imgSize, R, T, E, F,
		
		cv::CALIB_FIX_ASPECT_RATIO +
		cv::CALIB_ZERO_TANGENT_DIST +
		cv::CALIB_USE_INTRINSIC_GUESS +
		cv::CALIB_FIX_PRINCIPAL_POINT + 
		//cv::CALIB_FIX_FOCAL_LENGTH +
		cv::CALIB_RATIONAL_MODEL +
		//cv::CALIB_USE_EXTRINSIC_GUESS +
		cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
	std::cout << "done with RMS error=" << rms << std::endl;

	//////////////////////////////////////////////////// Calibrartion Ends ///////////////////////////////////////////////////////////

	// Bundle Adjustment. In each pair, world coordinate change from board coord to the first camera coord
	std::vector<Sophus::SE3<float>> T_esti_s; // estimated poses represented in Lie-Algebra. std::vector is an array for all pairs. There is a variable in the following text named 'T_esti', which will be push_back() into T_esti_s if it is not Nan in each for-loop round

	// For each pair
	for (int img_index = 0; img_index < nimages; img_index++)
	{
		std::stringstream depth_file1, depth_file2;
		depth_file1 << "Depth_Image_0" << "_" << allImgIds[img_index] << ".csv";
		depth_file2 << "Depth_Image_1" << "_" << allImgIds[img_index] << ".csv";
		std::cout << depth_file1.str() << " " << depth_file2.str() << "\n";

		Sophus::SE3<float > T_esti;	// contains R, T information

		float fx = (float)cameraMatrix[1].at<double>(0, 0);
		float fy = (float)cameraMatrix[1].at<double>(1, 1);
		float cx = (float)cameraMatrix[1].at<double>(0, 2);
		float cy = (float)cameraMatrix[1].at<double>(1, 2);

		//std::cout << cameraMatrix[1].at<double>(0, 0) << " " << cameraMatrix[1].at<double>(0, 1) << " " << cameraMatrix[1].at<double>(0, 2) << "\n" << cameraMatrix[1].at<double>(1, 0) << " " << cameraMatrix[1].at<double>(1, 1) << " " << cameraMatrix[1].at<double>(1, 2) << "\n" << cameraMatrix[1].at<double>(2, 0) << " " << cameraMatrix[1].at<double>(2, 1) << " " << cameraMatrix[1].at<double>(2, 2) << "\n";

		// p2f, p3f: image pixel coord points and 3-D points
		VecVector2f p2f;
		VecVector3f p3f;

		// Get depth for each pixel (we can also call it depth map)
		cv::Mat depth1 = extractDepthFile(depth_file1.str());	
		cv::Mat depth2 = extractDepthFile(depth_file2.str());

		cv::Mat points1 = depthmap2pointmap(depth1, cameraMatrix[0]);	// Code from Yajie Zhao. Given , generate point cloud in the world coordinate

		// Get data for pixel coord points and 3-D points
		for (int i = 0; i < (squaresX - 1) * (squaresY - 1); i++)
		{
			int xcoord = (int)imagePoints[0][img_index][i].x;
			int ycoord = (int)imagePoints[0][img_index][i].y;

			p3f.push_back(
				Eigen::Vector3f(
					points1.at<cv::Vec3f>(ycoord, xcoord)[0],
					points1.at<cv::Vec3f>(ycoord, xcoord)[1],
					points1.at<cv::Vec3f>(ycoord, xcoord)[2]
				)
			);

			//std::cout << ycoord << " " << xcoord << " " << points1.at<cv::Vec3f>(ycoord, xcoord)[0] << " " << points1.at<cv::Vec3f>(ycoord, xcoord)[1] << " " << points1.at<cv::Vec3f>(ycoord, xcoord)[2] << "\n";

			p2f.push_back(
				Eigen::Vector2f(
					imagePoints[1][img_index][i].x,
					imagePoints[1][img_index][i].y
				)
			);
		}

		//Gaussian-Newton iteration
		int iterations = 100;	// iteration times (Chinese: 迭代次数)
		float cost = 0, lastCost = 0;	// Cost for the current iteration and the previous iteration
		int nPoints = p3f.size();
		std::cout << "points: " << nPoints << std::endl;

		bool isNAN = false;

		for (int iter = 0; iter < iterations; iter++) {
			Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();	// Hessian matrix: J^T J
			Vector6f b = Vector6f::Zero();	// bias

			cost = 0;
			// compute cost
			for (int i = 0; i < nPoints; i++) {
				Eigen::Vector2f p2 = p2f[i];	// the i-th data
				Eigen::Vector3f p3 = p3f[i];	// the i-th data

				Eigen::Vector3f P = T_esti * p3;	// P = R * p3 + T. Transform to the world (first-camera) coord
				float x = P[0];
				float y = P[1];
				float z = P[2];

				Eigen::Vector2f p2_ = { fx * (x / z) + cx, fy * (y / z) + cy };	// K * P. K is intrinsic
				Eigen::Vector2f e = p2 - p2_;    //error = observation - estimation
				cost += (e[0] * e[0] + e[1] * e[1]);

				// compute jacobian
				Eigen::Matrix<float, 2, 6> J;
				J(0, 0) = - (fx / z);
				J(0, 1) = 0;
				J(0, 2) = (fx * x / (z * z));
				J(0, 3) = (fx * x * y / (z * z));
				J(0, 4) = -(fx * x * x / (z * z) + fx);
				J(0, 5) = (fx * y / z);
				J(1, 0) = 0;
				J(1, 1) = - (fy / z);
				J(1, 2) = (fy * y / (z * z));
				J(1, 3) = (fy * y * y / (z * z) + fy);
				J(1, 4) = - (fy * x * y / (z * z));
				J(1, 5) = - (fy * x / z);

				H += J.transpose() * J;
				b += - J.transpose() * e;
			}

			// solve dx
			Vector6f dx;
			// ldlt decomposition. Use built-in function in Eigen math toolkit. https://www.alglib.net/matrixops/symmetric/ldlt.php
			dx = H.ldlt().solve(b);

			if (isnan(dx[0])) {
				std::cout << "result is nan!" << std::endl;
				isNAN = true;
				break;
			}

			if (iter > 0 && cost >= lastCost) {
				// The cost increases, which means that the estimation is not good or over-fitting
				std::cout << "cost: " << cost << ", last cost: " << lastCost << std::endl;
				break;
			}

			// update pose estimation
			T_esti = Sophus::SE3<float>::exp(dx) * T_esti;
			lastCost = cost;
			std::cout << "iteration " << iter << " cost=" << std::cout.precision(12) << cost << std::endl;
			
		}

		if (!isNAN)
		{
			std::cout << "estimated pose: \n" << T_esti.matrix() << std::endl;	// output the result

			T_esti_s.push_back(T_esti);	// This step is useless but I still want to keep it. Because I intended to get the average pose from all the results, however, I found that the best result from one pair is better than the average result.
		}
		
	}

	return;
}

int main() {
	std::vector<std::string> list;
	int k = 0;
	while (k < 30) {
		std::stringstream f1, f2;
		f1 << "cam0_" << k << ".png";
		list.push_back(f1.str());
		f2 << "cam1_" << k << ".png";
		list.push_back(f2.str());
		k++;
	}

	StereoCalibCharuco(list, 5, 7, 0.1375, 0.0826);	// measured with ruler, do not change

	return 0;
}

cv::Mat extractDepthFile(const std::string& depthName)
{
	std::ifstream inFile(depthName, std::ios::in);
	std::string lineStr;
	std::vector<uint16_t> oneArray;
	while (getline(inFile, lineStr))
	{
		std::stringstream ss(lineStr);
		std::string str;
		// Separate by ',' (because it reads from CSV file)
		while (getline(ss, str, ','))
		{
			oneArray.push_back((uint16_t)std::stoi(str));
		}
	}

	cv::Mat timg = cv::Mat(oneArray);// change vector to one row Mat
	std::cout << timg.size() << "\n";
	cv::Mat image2 = timg.reshape(1, 480).clone();	// reshape to 640*480

	std::cout << image2.cols << " " << image2.rows << "\n";

	return image2;
}