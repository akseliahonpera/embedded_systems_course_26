#include "CSVFileWriter.hpp"
#include <iostream>

namespace CSV
{

CSVFileWriter::CSVFileWriter(const std::string &outputFile, const std::vector<std::string> &fieldNames)
{
    value_counter = 0;
    fields = fieldNames;
    bool append_existing_file = false;


    std::stringstream fields_ss;
    for (size_t i = 0; i < fields.size(); i++) {
        fields_ss << fields[i] << (i == fields.size()-1 ? "" : ",");
    }
    std::string fields_str = fields_ss.str();

    std::ifstream in_file(outputFile);
    if (in_file.is_open()) {
        std::string fields_from_file;
        std::getline(in_file, fields_from_file);

        if (fields_from_file == fields_str) {
            append_existing_file = true;
        }
    }
    in_file.close();

    if (append_existing_file) {
        file = std::ofstream(outputFile, std::ios::app);
    } else {
        file = std::ofstream(outputFile);
        file << fields_str << "\n";
    }

    if (!file.is_open()) {
        std::cout << "Cannot open file: " << outputFile << std::endl;
    }
}

CSVFileWriter::~CSVFileWriter()
{
    if (file.is_open()) {
        file.close();
    }
}

};
