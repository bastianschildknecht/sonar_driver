#pragma once


#include <sonar_driver/sonardevices/SonarImage.h>


class OculusSonarImage : public SonarImage
{
public:
    OculusSonarImage(){};
    ~OculusSonarImage(){};
    uint32_t pingStartTime = 0;
    double sonarFrequency = 0.0;
    double temperature = 0.0;
    double pressure = 0.0;
};

class OculusSonarImage2 : public SonarImage
{
public:
    OculusSonarImage2(){};
    ~OculusSonarImage2(){};
    double pingStartTime = 0.0;
    double sonarFrequency = 0.0;
    double temperature = 0.0;
    double pressure = 0.0;
    double heading = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
};
