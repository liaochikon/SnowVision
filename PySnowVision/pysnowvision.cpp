#include "pysnowvision.h"
#include "triangulation.h"

namespace py = pybind11;

void RecordVideo(std::string video_name, std::vector<int> camera_name_list, std::vector<int> resolution, int fps, int wait_sec)
{
	std::vector<cv::VideoCapture> cap_list;
	std::vector<cv::VideoWriter> out_list;

	for (int i = 0; i < camera_name_list.size(); i++)
	{
		if (camera_name_list[i] < 0)
		{
			std::cout << "Cap" << std::to_string(i) << "is skiped." << "\n";
			continue;
		}
		cv::VideoCapture cap_temp = cv::VideoCapture(i);
		if (!cap_temp.isOpened())
		{
			std::cout << "Cap" << std::to_string(i) << "is not valid." << "\n";
			continue;
		}
		std::cout << "Cap" << std::to_string(i) << "is valid." << "\n";
		cap_temp.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]);
		cap_temp.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1]);
		int w = cap_temp.get(cv::CAP_PROP_FRAME_WIDTH);
		int h = cap_temp.get(cv::CAP_PROP_FRAME_HEIGHT);
		cap_list.push_back(cap_temp);
		out_list.push_back(cv::VideoWriter(video_name + std::to_string(camera_name_list[i]) + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(w, h), true));
	}
	cv::Mat img_temp;
	while (true)
	{
		int i = 0;
		std::vector<cv::VideoCapture>::iterator cap_list_ptr;
		std::vector<cv::VideoWriter>::iterator out_list_ptr;
		for (cap_list_ptr = cap_list.begin(), out_list_ptr = out_list.begin();
			cap_list_ptr < cap_list.end() && out_list_ptr < out_list.end();
			cap_list_ptr++, out_list_ptr++, i++)
		{
			std::string window_name = "img" + std::to_string(i);
			cap_list_ptr->read(img_temp);
			out_list_ptr->write(img_temp);
			cv::circle(img_temp, cv::Point((int)(resolution[0] / 2), (int)(resolution[1] / 2)), 10, cv::Scalar(255, 255, 255), cv::FILLED, 1, 0);
			cv::imshow(window_name, img_temp);
		}
		char c = (char)cv::waitKey(wait_sec);
		if (c == 27) break;
	}
	return;
}

PYBIND11_MODULE(snowvision, m) 
{
    m.doc() = "pybind11 numpy plugin";
	m.def("RecordVideo", &RecordVideo);

	py::class_<CameraGroup>(m, "CameraGroup")
		.def(py::init<std::string>())
		.def("Triangulation", &CameraGroup::Triangulation)
		.def("AddCameraPoints", &CameraGroup::AddCameraPoints)
		.def("ClearCameraPoints", &CameraGroup::ClearCameraPoints)
		.def("ClearTriangulatedPoints", &CameraGroup::ClearTriangulatedPoints)
		.def("TriangulationCondense", &CameraGroup::TriangulationCondense)
		.def("TriangulationWorkspaceFilter", &CameraGroup::TriangulationWorkspaceFilter)
		.def_readonly("CameraCount", &CameraGroup::CameraCount)
		.def_readonly("CameraInfo", &CameraGroup::CameraInfo)
		.def_readonly("TriangulatedScore", &CameraGroup::TriangulatedScore)
		.def_readwrite("CameraPoints", &CameraGroup::CameraPoints)
		.def_readwrite("TriangulatedPoints", &CameraGroup::TriangulatedPoints);
    
    py::class_<Camera>(m, "Camera")
        .def(py::init<>())
        .def_readonly("K", &Camera::K)
        .def_readonly("K_Inverse", &Camera::K_Inverse)
        .def_readonly("Rt", &Camera::Rt)
        .def_readonly("R", &Camera::R)
        .def_readonly("t", &Camera::t)
        .def_readonly("D", &Camera::D);
}