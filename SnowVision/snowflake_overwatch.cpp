#include "SnowVision.h"
#include "FeatureFinder.h"
#include "BasicClasses.h"
#include "Triangulation.h"
#include "record.h"

double clockToMilliseconds(clock_t ticks) {
    return (ticks / (double)CLOCKS_PER_SEC) * 1000.0;
}

int main()
{
    std::vector<int> l{ -1, 1, 3, 0, 2 };
    std::vector<int> r{ 800, 450 };
    RecordVideo("sssss", l, r, 30, 1);
    /*CameraGroup a("output.txt");
    std::cout << a.CameraCount << "\n";
    std::cout << a.CameraInfo[0].K << "\n";
    std::cout << a.CameraInfo[0].Rt << "\n";
    std::cout << a.CameraInfo[0].D << "\n";
    std::cout << a.CameraInfo[1].K << "\n";
    std::cout << a.CameraInfo[1].Rt << "\n";
    std::cout << a.CameraInfo[1].D << "\n";

    std::vector<cv::VideoCapture> cap_array;
    int videolength = Load_Videos(cap_array, "point", a.CameraCount);

    for (int i = 0; i < 100; i++)
    {
        clock_t beginFrame = clock();

        std::vector<std::vector<Eigen::Vector2d>> camerapoints;
        std::vector<Eigen::Vector3d> TriangulatedPoints;

        cv::Mat img_temp;
        cv::Mat img_undist;
        for (int j = 0; j < a.CameraCount; j++)
        {
            std::vector<Eigen::Vector2d> points;
            std::string window_name = "img" + std::to_string(j);
            cap_array[j].read(img_temp);
            cv::cvtColor(img_temp, img_temp, cv::COLOR_BGR2GRAY);
            cv::threshold(img_temp, img_temp, 240, 255, cv::THRESH_BINARY);
            cv::undistort(img_temp, img_undist, a.CameraInfo[j].K_Mat, a.CameraInfo[j].D_Mat);
            cv::GaussianBlur(img_undist, img_undist, cv::Size(15, 15), 15, 0, 4);
            cv::imshow(window_name, img_undist);
            Find_Point(points, img_undist);
            camerapoints.push_back(points);
        }
        
        ImagePoints2TriangulatedPoints(TriangulatedPoints, camerapoints, a, 0.01);
        
        std::cout << "TriangulatedPoints_ptr" << "\n";
        std::vector<Eigen::Vector3d>::iterator TriangulatedPoints_ptr;
        for (TriangulatedPoints_ptr = TriangulatedPoints.begin(); TriangulatedPoints_ptr < TriangulatedPoints.end(); TriangulatedPoints_ptr++)
        {
            std::cout << *TriangulatedPoints_ptr << "\n";
            std::cout << "\n";
        }

        clock_t endFrame = clock();
        std::cout << endFrame - beginFrame << "\n";

        char c = (char)cv::waitKey(1);
        if (c == 27) break;
    }*/
    return 0;
}