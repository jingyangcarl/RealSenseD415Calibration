#include <librealsense2/rs.hpp>
using rs2::context;

#include <iostream>
using std::cout;
using std::endl;

int main() {
	rs2::context context;

	cout << "--- Camera Detection ---" << context.query_all_sensors().size() << endl;

	for (auto device : context.query_devices()) {
		cout << "Camera Info Asic Serial Number: " << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << endl;
		cout << "Camera Info Name" << device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << endl;
	}
}