#pragma once

#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <sonar_driver/ezsocket/ezsocket.hxx>
#include <sonar_driver/ezsocket/socketWorker.hxx>
#include <sonar_driver/sonardevices/OculusMessages.h>

#include <sonar_driver/sonardevices/SonarImage.h>

enum SonarState
{
    Ready,
    Connected,
    SocketError
};


class Sonar;
using SonarCallback = std::function<void(Sonar*, SonarImage*)>;
// typedef void (*SonarCallback)(Sonar *sonar, SonarImage *sonarImage);

class Sonar
{
public:
    Sonar();
    virtual ~Sonar() = 0;

    // Find sonar via UDP broadcast signal and connect to any sonar (blocking)
    virtual void findAndConnect() = 0;

    // Connect to sonar at specified IPv4 address (blocking)
    virtual void connect(const std::string& address) = 0;

    // Disconnect from sonar (blocking)
    virtual void disconnect() = 0;

    // Register a callback function to be called, when an image is ready (blocking)
    virtual void registerCallback(SonarCallback callback);

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
    virtual const std::string& getLocation() = 0;

    // Get the current fire mode of the sonar device
    virtual uint8_t getFireMode();

    // Get the current operating frequency of the sonar (Hz)
    virtual double getOperatingFrequency() = 0;

    // Get the current ping rate of the sonar (Hz)
    virtual uint8_t getPingRate();

    // Get the operational beam count of the sonar
    virtual uint16_t getBeamCount() = 0;

    // Get the beam separation of the sonar (degrees)
    virtual double getBeamSeparation() = 0;

    // Get the absolute minimum range of the sonar (meters)
    virtual double getMinimumRange() = 0;

    // Get the absolute maximum range of the sonar (meters)
    virtual double getMaximumRange() = 0;

    // Get the current set range of the sonar (meters)
    virtual double getCurrentRange();

    // Get the current range resolution of the sonar (meters)
    virtual double getRangeResolution() = 0;

    // Get the number of range bins.
    virtual uint32_t getRangeBinCount() = 0;

    // Get the current horizontal field of view of the sonar (degrees)
    virtual double getHorzFOV() = 0;

    // Get the current vertical field of view of the sonar (degrees)
    virtual double getVertFOV() = 0;

    // Get the current angular resolution of the sonar (degrees)
    virtual double getAngularResolution() = 0;

    // Get the current gain of the sonar (%)
    virtual double getCurrentGain();

    // Get whether the sonar is currently in gain assist mode
    virtual bool gainAssistEnabled();

    // Get the currently used gamma correction value
    virtual uint8_t getGamma();

    // Get the currently used speed of sound value (m/s)
    virtual double getSpeedOfSound();

    // Get the currently used salinity value (ppt)
    virtual double getSalinity();

    // Get the currently used temperature value (degC)
    virtual double getTemperature();

    // Get the currently used pressure value (bar)
    virtual double getPressure();

    // Get the set network speed limit (Mbps)
    virtual uint8_t getNetworkSpeedLimit();
    
    // Get the bearing table (deg)
    virtual std::vector<int16_t> getBearingTable() = 0;

    virtual std::string getDeviceName() = 0;


protected:
    SonarState state;
    std::vector<SonarCallback> *callbacks;
    std::thread *callbackThread;
    std::mutex *callbackMutex;
    SonarImage *lastImage;

    bool callbackThreadStarted;
    volatile bool callbackThreadActive;

    int fireMode;
    bool gainAssistActive;

    double currRange;
    double currGain;
    double speedOfSound;
    double salinity;
    double temperature;
    double pressure;
    uint8_t gamma;
    uint8_t netSpeedLimit;
    int pingRate;

    virtual void invokeCallbacks() = 0;
};