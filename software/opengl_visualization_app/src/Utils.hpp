#pragma once

#include <string>
#include <vector>

namespace Utils
{

std::string readTextFile(std::string filepath);
std::vector<std::string> splitSpring(const std::string &str, char splitter);
float getTimeStamp();

}
