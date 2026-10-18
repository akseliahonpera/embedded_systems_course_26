#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <sstream>

namespace CSV
{

class CSVFileWriter
{
public:
    CSVFileWriter(const std::string &outputFile, const std::vector<std::string> &fieldNames);
    ~CSVFileWriter();


    template<typename... Rest>
    void writeValues(Rest... rest) {
        std::stringstream ss;

        printValues(ss, rest...);

        file << ss.str();
    }

    std::vector<std::string> getFieldNames() const {
        return fields;
    }

private:
    char delimitter() {
        if (++value_counter == fields.size()) {
            value_counter = 0;
            return '\n';
        }
        return ',';
    }

    void printValues(std::stringstream &ss) { }

    template<typename T, typename... Rest>
    void printValues(std::stringstream &ss, T var, Rest... rest) {
        ss << var << delimitter();

        printValues(ss, rest...);
    }

    size_t value_counter;

    std::ofstream file;
    std::vector<std::string> fields;
};

};
