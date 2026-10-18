#include "DataSampler.hpp"
#include <algorithm>
#include <iostream>

constexpr double TIME_WINDOW = 10.0;
constexpr size_t MIN_ELEMENTS = 10;

void DataSampler::pushFrame(double t, const DataFrame &frame)
{
    if (frames.size() > MIN_ELEMENTS && t - frames[0].first > TIME_WINDOW) {
        frames.erase(std::remove_if(frames.begin(), frames.end(), [&] (auto &e) { return (t - e.first > TIME_WINDOW); }), frames.end());
    }

    frames.emplace_back(t, frame);
}

std::pair<double, DataFrame> DataSampler::getLastFrame()
{
    return frames.back();
}

std::pair<double, DataFrame> DataSampler::getFirstFrame()
{
    return frames.front();
}

DataFrame DataSampler::getSample(double t)
{
    if (frames.empty()) {
        return DataFrame();
    }

    if (t >= frames.back().first || frames.size() == 1) {
        return frames.back().second;
    }
    if (t <= frames.front().first) {
        return frames.front().second;
    }

    auto upper = std::upper_bound(frames.begin(), frames.end(), t, [] (const double &time, const std::pair<double, DataFrame> &frame) {return (time < frame.first); });
    auto lower = upper - 1;

    if (upper->first == lower->first) {
        return upper->second;
    }

    double frac = (t - lower->first) / (upper->first - lower->first);

    return DataFrame::interpolate(frac, lower->second, upper->second);
}

