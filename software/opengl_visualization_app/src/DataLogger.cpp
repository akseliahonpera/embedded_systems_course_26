#include "DataLogger.hpp"
#include "CSVFileWriter.hpp"

namespace Logging
{

DataLogger::DataLogger()
{
    log_coordinates = false;
    log_orientation = false;
    log_pressure = false;
    log_temperature = false;

    log_time = 0;
    samplerate = DEFAULT_SAMPLERATE;
}

void DataLogger::createWriter()
{
    std::vector<std::string> fields;

    if (log_orientation) {
        fields.emplace_back("roll");
        fields.emplace_back("pitch");
        fields.emplace_back("yaw");
    }
    if (log_coordinates) {
        fields.emplace_back("latitude");
        fields.emplace_back("longitude");
    }
    if (log_temperature) {
        fields.emplace_back("temperature");
    }
    if (log_pressure) {
        fields.emplace_back("pressure");
    }

    writer.reset(nullptr); //Manually free before creating new writer. Otherwise files might be locked
    writer = std::make_unique<CSV::CSVFileWriter>(output_file, fields);
}


void DataLogger::writeSample(double t, DataSampler &sampler)
{
    DataFrame frame = sampler.getSample(t);

    if (log_orientation) {
        writer->writeValues(frame.rotation.x, frame.rotation.y, frame.rotation.z);
    }
    if (log_coordinates) {
        writer->writeValues(frame.coordinates.x, frame.coordinates.y);
    }
    if (log_temperature) {
        writer->writeValues(frame.temperature);
    }
    if (log_pressure) {
        writer->writeValues(frame.pressure);
    }
}

void DataLogger::log(DataSampler &sampler)
{
    if (!writer) {
        return;
    }

    log_time = std::max(sampler.getFirstFrame().first, log_time);

    double end_time = sampler.getLastFrame().first;
    double delta_time = 1.0 / samplerate;

    while (log_time < end_time) {

        writeSample(log_time, sampler);

        log_time += delta_time;
    }
}

};
