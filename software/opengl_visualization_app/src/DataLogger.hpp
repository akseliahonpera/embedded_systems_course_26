#pragma once


#include "CSVFileWriter.hpp"
#include "DataSampler.hpp"
#include <memory>

namespace Logging
{

constexpr double DEFAULT_SAMPLERATE = 10;

class DataLogger
{
public:
    DataLogger();

    void log(DataSampler &sampler);

    void setOutputFile(const std::string &file)
    {
        output_file = file;
        createWriter();
    }
    void setSampleRate(double rt) {
        samplerate = rt;
    }
    void enableRotationLogging(bool e) {
        log_rotation = e;
        createWriter();
    }
    void enableCoordinatesLogging(bool e) {
        log_coordinates = e;
        createWriter();
    }
    void enablePressureLogging(bool e) {
        log_pressure = e;
        createWriter();
    }
    void enableTemperatureLogging(bool e) {
        log_temperature = e;
        createWriter();
    }
    void enableRelativeLocationLogging(bool e) {
        log_relative_location = e;
        createWriter();
    }

private:
    void createWriter();
    void writeSample(double t, DataSampler &sampler);

    bool log_rotation;
    bool log_coordinates;
    bool log_relative_location;
    bool log_pressure;
    bool log_temperature;

    std::unique_ptr<CSV::CSVFileWriter> writer;

    double log_time;
    double samplerate;

    std::string output_file;
};


};
