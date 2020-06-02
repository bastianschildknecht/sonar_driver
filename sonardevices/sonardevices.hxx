// File: sonardevices.hxx

#ifndef SONARDEVICES_HXX
#define SONARDEVICES_HXX

#include <cstdint>
#include <vector>
#include <ezsocket.hxx>

namespace SonarDevices
{
    /**
     * Generic definitions
     */

    enum SonarState
    {
        Ready,
        Connected,
        SocketError
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

    class Sonar
    {
    public:
        Sonar();
        virtual ~Sonar() = 0;

        // Find sonar via UDP broadcast signal and connect to any sonar (blocking)
        virtual void findAndConnect() = 0;

        // Connect to sonar at specified IPv4 address (blocking)
        virtual void connect(const char *address) = 0;

        // Disconnect from sonar (blocking)
        virtual void disconnect() = 0;

        // Register a callback function to be called, when an image is ready (non-blocking)
        virtual void registerCallback(void (*callback)(SonarImage *sonarImage));

        // Signal the sonar to aquire a new image (non-blocking)
        virtual void fire() = 0;

        // Retrieve the last sonar image captured
        // If no image is available NULL is returned
        virtual SonarImage *getLastImage() = 0;

        // Get the current connection state of the sonar
        virtual SonarState getState();

        // Get a string describing the current address or interface location of the sonar
        virtual const char *getLocation() = 0;

    protected:
        SonarState state;
        std::vector<void (*)(SonarImage *)> *callbacks;
    };

    /**
     * Blueprint subsea Oculus specific definitions
     */

    class OculusSonar : public Sonar
    {
    public:
        OculusSonar();
        ~OculusSonar() = 0;
        virtual void findAndConnect();
        virtual void connect(const char *address);
        virtual void disconnect();
        virtual void fire();
        virtual SonarImage *getLastImage();
        virtual const char *getLocation();

    protected:
        EZSocket::Socket *sonarUDPSocket;
        EZSocket::Socket *sonarTCPSocket;
        char *sonarAddress;
    };

#define OCULUS_CHECK_ID 0x4f53

#define OCULUS_UDP_PORT 52102
#define OCULUS_TCP_PORT 52100

    /**
    * Oculus M1200d specific definitions
    */

    class OculusM1200d : public OculusSonar
    {
    public:
        OculusM1200d();
        ~OculusM1200d();
    };

} // namespace SonarDevices

#endif