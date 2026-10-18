#pragma once

#include <vector>
#include "DataFrame.hpp"

class DataSampler
{
public:
    void pushFrame(double t, const DataFrame &frame);

    DataFrame getSample(double t);
    std::pair<double, DataFrame> getLastFrame();
    std::pair<double, DataFrame> getFirstFrame();

    size_t bufferedFrames() {
        return frames.size();
    }
private:
    std::vector<std::pair<double, DataFrame>> frames;
};
