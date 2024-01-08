#pragma once

#include <stdint.h>

class SonarImage
{
public:
    uint16_t imageWidth = 0;
    uint16_t imageHeight = 0;
    uint8_t *data  = nullptr;
    int16_t* bearingTable = nullptr;
};
