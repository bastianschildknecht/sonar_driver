// File: sonardevices.hxx

#ifndef SONARDEVICES_HXX
#define SONARDEVICES_HXX

#include <cstdint>

namespace SonarDevices
{
    class Sonar
    {
        public:
        Sonar();
        virtual ~Sonar() = 0;

        // Find sonar via UDP broadcast signal and connect to any sonar (blocking)
        virtual void findAndConnect();
        
        // Connect to sonar at specified IPv4 address (blocking)
        virtual void connect(const char * address);

        // Disconnect from sonar (blocking)
        virtual void disconnect();

        // Register a callback function to be called, when an image is ready (non-blocking)
        virtual void registerCallback(void (*callback)(SonarImage &sonarImage));

        // Signal the sonar to aquire a new image (non-blocking)
        virtual void fire();

        // Retrieve the last sonar image captured
        virtual SonarImage getLastImage();
    };

    class OculusM1200d : Sonar
    {
        public:
        OculusM1200d();
        ~OculusM1200d();
    };

    class SonarImage
    {
        public:
        SonarImage();
        ~SonarImage();
        uint16_t imageWidth;
        uint16_t imageHeight;
        uint8_t *data[];
    };
}

#endif