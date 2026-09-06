#include <iostream>
#include <cstdio>
#include <fstream>
#include "ObjLoader.hpp"
#include "Utils.hpp"

namespace ObjLoader
{


ObjModel loadObj(std::string filename)
{
    //Tämä paske ei validoi mitään
    //Paskalla filulla tulee kaatumaan

    ObjModel model;

    std::ifstream file(filename);
    std::string line;
    while (getline(file, line)) {
        if (line.length() < 2) {
            continue;
        }
        if (line[0] == 'v' && line[1] == 't') {
            float floats[2];
            std::sscanf(line.c_str(), "vt %f %f", &floats[0], &floats[1]);

            model.uvs.emplace_back(floats[0], floats[1]);
        } else if (line[0] == 'v' && line[1] == 'n') {
            float floats[3];
            std::sscanf(line.c_str(), "vn %f %f %f", &floats[0], &floats[1], &floats[2]);

            model.normals.emplace_back(floats[0], floats[1], floats[2]);
        } else if (line[0] == 'v') {
            float floats[3];
            std::sscanf(line.c_str(), "v %f %f %f", &floats[0], &floats[1], &floats[2]);

            model.vertices.emplace_back(floats[0], floats[1], floats[2]);
        } else if (line[0] == 'f') {
            auto parts = Utils::splitSpring(line, ' ');

            if (parts.size() < 4) {
                std::cout << "Bad face line" << std::endl;
                break;
            }
            for (auto part = parts.begin()+1; part < parts.end(); part++) {
                for (auto idx_str : Utils::splitSpring(*part, '/')) {
                    model.faces.push_back(std::atoi(idx_str.c_str())-1);
                }
            }
            model.num_of_faces += 1;
        }
    }
    return model;
}


}
