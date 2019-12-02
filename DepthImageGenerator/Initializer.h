#pragma once
#include <librealsense2/rs.hpp>
using namespace rs2;
#include <map>
#include <string>
#include <iostream>
using namespace std;

class Initializer {
public:
	Initializer(context& context, map<string, int>& asicSerialIndexMap, map<string, int>& serialIndexMap, map<int, colorizer>& colorizers, vector<pipeline>& pipelines);

	void AsicSerialIndexInitialization();
	void SerialIndexInitialization();
	void CameraInfoPrint();
	void CameraInitialization();

	bool isEnableStreamColor() const;
	bool isEnableStreamDepth() const;
	bool isEnableStreamInfrared() const;

private:
	context& context;
	map<string, int>& asicSerialIndexMap;
	map<string, int>& serialIndexMap;
	map<int, colorizer>& colorizers;
	vector<pipeline>& pipelines;

	bool enable_stream_color = false;
	bool enable_stream_depth = false;
	bool enable_stream_infrared = false;
};
