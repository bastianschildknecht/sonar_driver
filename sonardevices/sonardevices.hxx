// File: sonardevices.hxx

#ifndef SONARDEVICES_HXX
#define SONARDEVICES_HXX

#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <ezsocket.hxx>
#include <socketWorker.hxx>
#include <oculusMessages.hxx>

namespace SonarDevices
{
    /**
     * Generic definitions
     **/

    enum SonarState
    {
        Ready,
        Connected,
        SocketError
    };

    enum SonarImageType
    {
        SonarImageObject,
        OculusSonarImageObject,
        OculusSonarImage2Object
    };

    class SonarImage
    {
    public:
        SonarImage();
        ~SonarImage();
        SonarImageType imageType;
        uint16_t imageWidth;
        uint16_t imageHeight;
        uint8_t *data;
    };

    class OculusSonarImage : public SonarImage
    {
    public:
        OculusSonarImage();
        ~OculusSonarImage();
        uint32_t pingStartTime;
        double sonarFrequency;
        double temperature;
        double pressure;
    };

    class OculusSonarImage2 : public SonarImage
    {
    public:
        OculusSonarImage2();
        ~OculusSonarImage2();
        double pingStartTime;
        double sonarFrequency;
        double temperature;
        double pressure;
        double heading;
        double pitch;
        double roll;
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

        // Register a callback function to be called, when an image is ready (blocking)
        virtual void registerCallback(void (*callback)(SonarImage *sonarImage));

        // Configure the sonar for firing
        virtual void configure(int mode, double range, double gain, double speedOfSound, double salinity, bool gainAssist, uint8_t gamma, uint8_t netSpeedLimit) = 0;

        // Set the ping rate (image update frequency) for the sonar
        // returns the actual ping rate that was set
        virtual uint8_t setPingRate(uint8_t frequency) = 0;

        // Signal the sonar to aquire a new image (blocking)
        virtual void fire() = 0;

        // Retrieve the last sonar image captured
        // If no image is available an empty image is returned
        virtual SonarImage getLastImage();

        // Get the current connection state of the sonar
        virtual SonarState getState();

        // Get a string describing the current address or interface location of the sonar
        virtual const char *getLocation() = 0;

    protected:
        SonarState state;
        std::vector<void (*)(SonarImage *)> *callbacks;
        std::thread *callbackThread;
        std::mutex *callbackMutex;
        SonarImage *lastImage;
        bool callbackThreadStarted;
        volatile bool callbackThreadActive;
        int fireMode;
        double currRange;
        double currGain;
        double speedOfSound;
        double salinity;
        bool gainAssistActive;
        uint8_t gamma;
        uint8_t netSpeedLimit;
        int pingRate;
        virtual void invokeCallbacks() = 0;
    };

    /**
     * Blueprint subsea Oculus specific definitions
     **/
#define SONAR_READ_BUFFER_SIZE 200000
#define SONAR_WRITE_BUFFER_SIZE 200000

    class OculusSonar : public Sonar
    {
    public:
        OculusSonar();
        ~OculusSonar();
        virtual void findAndConnect();
        virtual void connect(const char *address);
        virtual void disconnect();
        virtual void configure(int mode, double range, double gain, double speedOfSound, double salinity, bool gainAssist, uint8_t gamma, uint8_t netSpeedLimit);
        virtual uint8_t setPingRate(uint8_t frequency);
        virtual void fire();
        virtual const char *getLocation();

    protected:
        EZSocket::Socket *sonarUDPSocket;
        EZSocket::Socket *sonarTCPSocket;
        char *sonarAddress;
        OculusMessages::PingRateType oculusPingRate;
        virtual void invokeCallbacks();
        virtual void processSimplePingResult(OculusMessages::OculusSimplePingResult *msg);
    };

#define OCULUS_UDP_PORT 52102
#define OCULUS_TCP_PORT 52100

    /**
    * Oculus M1200d specific definitions
    **/

    class OculusM1200d : public OculusSonar
    {
    public:
        OculusM1200d();
        ~OculusM1200d();
    };

} // namespace SonarDevices

#endif