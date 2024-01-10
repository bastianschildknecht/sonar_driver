#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

class SonarImage
{
public:
    SonarImage() : data(20000), bearingTable(1024) {
        bearingTable.resize(1024);
        data.resize(200000);
    };

    uint16_t imageWidth = 0;
    uint16_t imageHeight = 0;
    std::vector<uint8_t> data;
    std::vector<int16_t> bearingTable;
};
