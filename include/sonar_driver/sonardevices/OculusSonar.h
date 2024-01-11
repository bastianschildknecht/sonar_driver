
#pragma once

#include <sonar_driver/sonardevices/Sonar.h>
#include <sonar_driver/sonardevices/OculusSonarImage.h>

#include <memory.h>
#include <string.h>

#define SONAR_READ_BUFFER_SIZE 200000
#define SONAR_WRITE_BUFFER_SIZE 200000

#define OCULUS_UDP_PORT 52102
#define OCULUS_TCP_PORT 52100

class OculusSonar : public Sonar
{
public:
    OculusSonar();
    OculusSonar(std::shared_ptr<sensor_msgs::msg::Image> messagePointer);
    ~OculusSonar();

    void findAndConnect() override;
    void connect(const std::string& address) override;
    void disconnect() override;

    void configure(int mode, double range, double gain, double speedOfSound, double salinity, bool gainAssist, uint8_t gamma, uint8_t netSpeedLimit) override;
    uint8_t setPingRate(uint8_t frequency) override;

    void fire() override;

    const std::string& getLocation() override;
    double getOperatingFrequency() override;
    double getBeamSeparation() override;
    double getMinimumRange() override;
    double getMaximumRange() override;
    double getRangeResolution() override;
    double getHorzFOV() override;
    double getVertFOV() override;
    double getAngularResolution() override;
    uint32_t getRangeBinCount() override;
    uint16_t getBeamCount() override;
    std::string getDeviceName() override;
    std::vector<int16_t> getBearingTable() override;

protected:
    std::shared_ptr<EZSocket::Socket> sonarUDPSocket;
    std::shared_ptr<EZSocket::Socket> sonarTCPSocket;
    std::string sonarAddress;

    OculusMessages::OculusPartNumberType partNumber;
    OculusMessages::PingRateType oculusPingRate;

    double operatingFrequency;
    double rangeResolution;
    uint16_t beams;
    uint32_t rangeBinCount;

    virtual void invokeCallbacks() override;
    void processSimplePingResult(OculusMessages::OculusSimplePingResult *msg);
    OculusMessages::OculusPartNumberType determinePartNumber(char *broadcastMessage);
};

