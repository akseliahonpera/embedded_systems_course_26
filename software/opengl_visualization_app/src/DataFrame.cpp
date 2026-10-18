#include "DataFrame.hpp"
#include <iostream>

void DataFrame::print()
{
    std::cout << "coordinates: " << coordinates.x << ", " << coordinates.y << std::endl;
    std::cout << "rotation: " << rotation.x << ", " << rotation.y << ", " << rotation.z << std::endl;
    std::cout << "pressure: " << pressure << std::endl;
    std::cout << "temperature: " << temperature << std::endl;
}
