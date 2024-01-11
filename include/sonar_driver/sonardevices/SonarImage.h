#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

class SonarImage
{
public:
    SonarImage() {
        bearingTable->resize(1024);
        data->resize(200000);
    };

    uint16_t imageWidth = 0;
    uint16_t imageHeight = 0;
    std::unique_ptr<std::vector<uint8_t>> data;
    std::unique_ptr<std::vector<int16_t>> bearingTable;
};
