#include <iostream>
#include <fstream>
#include <chrono>
#include "Utils.hpp"

namespace Utils
{

std::string readTextFile(std::string path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "Error! Can't read file " << path << std::endl;
        return "";
    }
    std::string str;
    std::string line;
    while(std::getline(file, line)){
        str += line + "\n";
    }
    return str;
}


std::vector<std::string> splitSpring(const std::string &str, char splitter)
{
    std::vector<std::string> substrings;
    size_t pos = 0, new_pos;
    while ((new_pos = str.find(splitter, pos)) != std::string::npos) {
        substrings.push_back(str.substr(pos, new_pos-pos));
        pos = new_pos + 1;
    }
    if (pos < str.length())
        substrings.push_back(str.substr(pos, str.length()-pos));

    return substrings;
}

float getTimeStamp()
{
    using clock = std::chrono::steady_clock;

    uint64_t microseconds = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(clock::now().time_since_epoch()).count());

    static uint64_t first = microseconds;

    //Use 64-bit floating point when calculating in microseconds scale
    //Because timers uses microseconds, 32-bit floating point might not have optimal precision
    double dp = static_cast<double>(microseconds - first) / 1000000.0;

    //After microseconds has been converted to seconds, we can more safely use 32-bit floats
    return static_cast<float>(dp);
}


}
