#pragma once

#include <memory>
#include <stdint.h>

class SonarImage
{
public:
    uint16_t imageWidth = 0;
    uint16_t imageHeight = 0;
    std::shared_ptr<uint8_t> data;
    std::shared_ptr<int16_t> bearingTable;
};
