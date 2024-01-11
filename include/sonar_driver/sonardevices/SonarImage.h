#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

class SonarImage
{
public:
    SonarImage() {
        std::unique_ptr<std::vector<uint8_t>> data = std::make_unique<std::vector<uint8_t>>(1024);
        std::unique_ptr<std::vector<int16_t>> bearingTable = std::make_unique<std::vector<int16_t>>(200000);
    };

    uint16_t imageWidth = 0;
    uint16_t imageHeight = 0;
    std::unique_ptr<std::vector<uint8_t>> data;
    std::unique_ptr<std::vector<int16_t>> bearingTable;
};
