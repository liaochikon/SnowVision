#include "SnowVision.h"

std::vector<std::string> Split(std::string fulltext, std::string split_key)
{
    std::vector<std::string> splittext;
    size_t pos = 0;
    std::string token;
    while ((pos = fulltext.find(split_key)) != std::string::npos) {
        token = fulltext.substr(0, pos);
        splittext.push_back(token);
        fulltext.erase(0, pos + split_key.length());
        if ((pos = fulltext.find(split_key)) == std::string::npos)
            splittext.push_back(fulltext);
    }
    return splittext;
}

std::vector<std::string> ReadLine(std::string fulltext)
{
    std::string split_key = "\n";
    std::vector<std::string> splittext;
    size_t pos = 0;
    std::string token;
    while ((pos = fulltext.find(split_key)) != std::string::npos) {
        token = fulltext.substr(0, pos);
        splittext.push_back(token);
        fulltext.erase(0, pos + split_key.length());
    }
    return splittext;
}

std::string txt2String(std::string filepath) {
    std::ifstream ifs;
    char buffer[10000] = { 0 };
    ifs.open(filepath);
    if (ifs.is_open()) {
        ifs.read(buffer, sizeof(buffer));
        ifs.close();
    }
    return buffer;
}

void Read_2DArray(Eigen::MatrixXd& mat, std::string line)
{
    std::vector<std::string> arr_str = Split(Split(line, "=")[1], ",");
    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            mat(i, j) = std::stof(arr_str[i * mat.cols() + j]);
        }
    }
}

int Load_Images(std::vector<cv::VideoCapture>& cap_array, std::string imagepath, int cameracount)
{
    int isimgexist = 0;
    for (int i = 0; i < cameracount; i++)
    {
        std::string image_name = imagepath + std::to_string(i) + ".avi";
        cv::VideoCapture vc = cv::VideoCapture(image_name);
        if (!vc.isOpened())
        {
            std::cout << "Can't get image:" << image_name << "\n";
            isimgexist = 0;
            break;
        }
        cap_array.push_back(vc);
        std::cout << "Get Image:" << image_name << "\n";
        //int v_w = (int)cap_array[i].get(CAP_PROP_FRAME_WIDTH);
        //int v_h = (int)cap_array[i].get(CAP_PROP_FRAME_HEIGHT);
        isimgexist = 1;
    }
    return isimgexist;
}

int Load_Videos(std::vector<cv::VideoCapture>& cap_array, std::string videopath, int cameracount)
{
    int videolength = -1;
    for (int i = 0; i < cameracount; i++)
    {
        std::string video_name = videopath + std::to_string(i) + ".avi";
        std::cout << "Get Video:" << video_name << "\n";
        cap_array.push_back(cv::VideoCapture(video_name));
        int length_temp = (int)cap_array[i].get(cv::CAP_PROP_FRAME_COUNT);
        if (videolength > length_temp || i == 0)
            videolength = length_temp;
        //int v_w = (int)cap_array[i].get(CAP_PROP_FRAME_WIDTH);
        //int v_h = (int)cap_array[i].get(CAP_PROP_FRAME_HEIGHT);
    }
    return videolength;
}