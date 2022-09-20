#include "fileprocessor.h"

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