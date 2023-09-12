// File: sonardevices.cxx

#include <vector>
#include <cstring>
#include <memory.h>
#include <sonar_driver/ezsocket/ezsocket.hxx>
#include <sonar_driver/sonardevices/sonardevices.hxx>
#include <sonar_driver/sonardevices/oculusMessages.hxx>

using namespace EZSocket;
using namespace SonarDevices;
using namespace OculusMessages;

Sonar::Sonar()
{
    fireMode = 1;
    currRange = 20.0;
    currGain = 1.0;
    speedOfSound = 0.0;
    salinity = 0.0;
    temperature = 0.0;
    pressure = 0.0;
    gainAssistActive = false;
    gamma = 0;
    netSpeedLimit = 255;
    pingRate = 0;
    callbacks = new std::vector<SonarCallback>();
    callbackMutex = new std::mutex();
    callbackThreadStarted = false;
    callbackThreadActive = false;
    lastImage = new SonarImage();
    state = SonarState::Ready;
}

Sonar::~Sonar()
{
    if (callbackThreadStarted)
    {
        callbackThreadActive = false;
        callbackThread->join();
        delete callbackThread;
        callbackThread = nullptr;
        callbackThreadStarted = false;
    }
    delete callbacks;
    callbacks = nullptr;
    delete callbackMutex;
    callbackMutex = nullptr;
    delete lastImage;
    lastImage = nullptr;
}

void Sonar::registerCallback(SonarCallback callback)
{
    std::lock_guard<std::mutex> lock(*callbackMutex);
    callbacks->push_back(callback);
}

SonarState Sonar::getState()
{
    return state;
}

SonarImage Sonar::getLastImage()
{
    return *lastImage;
}

uint8_t Sonar::getFireMode()
{
    return fireMode;
}

uint8_t Sonar::getPingRate()
{
    return pingRate;
}

double Sonar::getCurrentRange()
{
    return currRange;
}

double Sonar::getCurrentGain()
{
    return currGain;
}

bool Sonar::gainAssistEnabled()
{
    return gainAssistActive;
}

uint8_t Sonar::getGamma()
{
    return gamma;
}

double Sonar::getSpeedOfSound()
{
    return speedOfSound;
}

double Sonar::getSalinity()
{
    return salinity;
}

double Sonar::getTemperature()
{
    return temperature;
}

double Sonar::getPressure()
{
    return pressure;
}

uint8_t Sonar::getNetworkSpeedLimit()
{
    return netSpeedLimit;
}

OculusSonar::OculusSonar() : Sonar()
{
    partNumber = OculusPartNumberType::partNumberUndefined;
    sonarTCPSocket = new TCPSocket();
    sonarUDPSocket = new UDPSocket();
    sonarAddress = new char[STR_ADDRESS_LEN];
    oculusPingRate = PingRateType::pingRateStandby;
    operatingFrequency = 0.0;
    beams = 0;
    rangeBinCount = 0;
    rangeResolution = 0.0;
}

OculusSonar::~OculusSonar()
{
    delete sonarTCPSocket;
    sonarTCPSocket = nullptr;
    delete sonarUDPSocket;
    sonarUDPSocket = nullptr;
    delete[] sonarAddress;
    sonarAddress = nullptr;
}

void OculusSonar::findAndConnect()
{
    if (sonarUDPSocket->getState() == SocketState::Ready)
    {
        sonarUDPSocket->bindToAddress("ANY", OCULUS_UDP_PORT);
        if (sonarUDPSocket->getState() == SocketState::Bound)
        {
            char *buf = new char[256];
            if (sonarUDPSocket->waitForDataAndAddress(buf, 256, sonarAddress, STR_ADDRESS_LEN) > 0)
            {
                this->partNumber = determinePartNumber(buf);
                connect(sonarAddress);
            }
            delete[] buf;
            buf = nullptr;
        }
    }
}

OculusMessages::OculusPartNumberType OculusSonar::determinePartNumber(char *udpBroadcastMessage)
{
    // First check if the message is a valid Oculus message
    // The first two bytes should containt the Oculus Check ID
    uint16_t checkID = *(uint16_t *)udpBroadcastMessage;
    if (checkID != OCULUS_CHECK_ID)
    {
        return OculusPartNumberType::partNumberUndefined;
    }

    // The UDP broadcast message contains a OculusMessageHeader as
    // the first part of the message.
    OculusMessageHeader *oculusMessageHeader = (OculusMessageHeader *)udpBroadcastMessage;

    // The spare2 field contains the part number.
    uint16_t numericPartNumber = oculusMessageHeader->spare2;

    // Cast the short to a OculusPartNumberType
    OculusPartNumberType partNumber = static_cast<OculusPartNumberType>(numericPartNumber);

    return partNumber;
}

void OculusSonar::connect(const char *address)
{
    if (sonarTCPSocket->getState() == SocketState::Ready)
    {
        sonarTCPSocket->setReceiveBufferSize(200000);
        sonarTCPSocket->connectToHost(address, OCULUS_TCP_PORT);
        if (sonarTCPSocket->getState() == SocketState::Connected)
        {
            strncpy(sonarAddress, address, STR_ADDRESS_LEN);
            callbackThreadActive = true;
            callbackThread = new std::thread([this] { this->invokeCallbacks(); });
            callbackThreadStarted = true;
            state = SonarState::Connected;
        }
    }
}

void OculusSonar::disconnect()
{
    if (callbackThreadStarted)
    {
        callbackThreadActive = false;
        callbackThread->join();
        delete callbackThread;
        callbackThread = nullptr;
        callbackThreadStarted = false;
    }
    sonarUDPSocket->disconnect();
    sonarTCPSocket->disconnect();
    if (sonarUDPSocket->getState() == SocketState::Ready && sonarTCPSocket->getState() == SocketState::Ready)
    {
        state = SonarState::Ready;
    }
    else
    {
        state = SonarState::SocketError;
    }
}

void OculusSonar::configure(int mode, double range, double gain, double speedOfSound, double salinity, bool gainAssist, uint8_t gamma, uint8_t netSpeedLimit)
{
    fireMode = mode;
    currRange = range;
    currGain = gain;
    this->speedOfSound = speedOfSound;
    this->salinity = salinity;
    gainAssistActive = gainAssist;
    this->gamma = gamma;
    this->netSpeedLimit = netSpeedLimit;
}

uint8_t OculusSonar::setPingRate(uint8_t frequency)
{
    if (frequency == 0)
    {
        pingRate = 0;
        oculusPingRate = PingRateType::pingRateStandby;
    }
    else if (frequency <= 2)
    {
        pingRate = 2;
        oculusPingRate = PingRateType::pingRateLowest;
    }
    else if (frequency <= 5)
    {
        pingRate = 5;
        oculusPingRate = PingRateType::pingRateLow;
    }
    else if (frequency <= 10)
    {
        pingRate = 10;
        oculusPingRate = PingRateType::pingRateNormal;
    }
    else if (frequency <= 15)
    {
        pingRate = 15;
        oculusPingRate = PingRateType::pingRateHigh;
    }
    else
    {
        pingRate = 40;
        oculusPingRate = PingRateType::pingRateHighest;
    }
    return pingRate;
}

void OculusSonar::fire()
{
    OculusSimpleFireMessage sfm;
    memset(&sfm, 0, sizeof(OculusSimpleFireMessage));

    sfm.head.msgId = OculusMessageType::messageSimpleFire;
    sfm.head.srcDeviceId = 0;
    sfm.head.dstDeviceId = 0;
    sfm.head.oculusId = OCULUS_CHECK_ID;

    uint8_t flags = 0;
    flags |= 0x01; // Interpret range as meters
    flags |= 0x08; // Send simple return message
    flags |= 0x40; // Enable 512 beams

    if (gainAssistActive)
    {
        flags |= 0x10; // Enable gain assist
    }

    sfm.flags = flags;
    sfm.gammaCorrection = gamma;
    sfm.pingRate = oculusPingRate;
    sfm.networkSpeed = netSpeedLimit;
    sfm.masterMode = fireMode;
    sfm.range = currRange;
    sfm.gainPercent = currGain;
    sfm.speedOfSound = speedOfSound;
    sfm.salinity = salinity;

    sonarTCPSocket->writeData(&sfm, sizeof(OculusSimpleFireMessage));
}

void OculusSonar::invokeCallbacks()
{

    uint64_t bytesAvailable;
    uint64_t rxBuffSize = 2048;
    uint64_t rxBuffPos = 0;
    uint8_t *rxBuff = (uint8_t *)malloc(rxBuffSize);

    int32_t bytesRead;
    uint64_t currPktSize;

    while (callbackThreadActive)
    {
        // Check for received data
        bytesAvailable = sonarTCPSocket->bytesAvailable();
        if (bytesAvailable > 0)
        {
            // Check if buffer is big enough and expand if required
            if (bytesAvailable > rxBuffSize - rxBuffPos)
            {
                rxBuffSize = bytesAvailable + rxBuffPos;
                rxBuff = (uint8_t *)realloc(rxBuff, rxBuffSize);
            }

            bytesRead = sonarTCPSocket->readData(&rxBuff[rxBuffPos], bytesAvailable);
            // Check for socket errors or disconnects
            if (bytesRead < 0)
            {
                // Socket error or disconnect, exit thread
                break;
            }
            rxBuffPos += bytesRead;

            // Process received data
            currPktSize = sizeof(OculusMessageHeader);
            if (rxBuffPos >= currPktSize)
            {
                OculusMessageHeader *omh = (OculusMessageHeader *)rxBuff;

                // Check if data is valid via oculus ID
                if (omh->oculusId != OCULUS_CHECK_ID)
                {
                    // Data invalid, flush buffer
                    rxBuffPos = 0;
                    continue;
                }

                currPktSize += omh->payloadSize;

                // Check if payload is also in the buffer and process message
                if (rxBuffPos >= currPktSize)
                {
                    // Check which message type was received
                    switch (omh->msgId)
                    {
                    case OculusMessageType::messageSimplePingResult:
                        // Received result of ping
                        processSimplePingResult((OculusSimplePingResult *)omh);
                        break;
                    case OculusMessageType::messageUserConfig:
                        // Received user config
                        break;
                    case OculusMessageType::messageDummy:
                        // Received dummy message
                        break;
                    default:
                        // Unrecognized message
                        break;
                    }

                    // If there is additional data in buffer shift it to the front
                    memmove(rxBuff, &rxBuff[currPktSize], rxBuffPos - currPktSize);
                    rxBuffPos -= currPktSize;
                }
            }
        }
    }

    // Free memory
    free(rxBuff);
    rxBuff = nullptr;
}

void OculusSonar::processSimplePingResult(OculusSimplePingResult *ospr)
{
    uint8_t *startAddress = (uint8_t *)ospr;

    uint32_t imageSize;
    uint32_t imageOffset;
    uint16_t beams;
    uint16_t ranges;

    uint16_t version = ospr->fireMessage.head.msgVersion;
    
    OculusSimplePingResult2 *ospr2 = (OculusSimplePingResult2 *)ospr;

    // Check message version
    switch (version)
    {
    case 2:
        imageSize = ospr2->imageSize;
        imageOffset = ospr2->imageOffset;
        beams = ospr2->nBeams;
        ranges = ospr2->nRanges;
        this->rangeResolution = ospr2->rangeResolution;
        break;
    default:
        imageSize = ospr->imageSize;
        imageOffset = ospr->imageOffset;
        beams = ospr->nBeams;
        ranges = ospr->nRanges;
        this->rangeResolution = ospr->rangeResolution;
        break;
    }

    // Remember the beams and range bin count
    this->beams = beams;
    this->rangeBinCount = ranges;

    // Check if image size is correct
    if (ospr->fireMessage.head.payloadSize + sizeof(OculusMessageHeader) == imageOffset + imageSize)
    {
        // Copy image into sonar image
        SonarImage *img;

        switch (version)
        {
        case 2:
        {
            OculusSonarImage2 *osimg2 = new OculusSonarImage2();
            osimg2->pingStartTime = ospr2->pingStartTime;
            osimg2->sonarFrequency = ospr2->frequency;
            this->operatingFrequency = ospr2->frequency;
            osimg2->temperature = ospr2->temperature;
            this->temperature = ospr2->temperature;
            osimg2->pressure = ospr2->pressure;
            this->pressure = ospr2->pressure;
            osimg2->heading = ospr2->heading;
            osimg2->pitch = ospr2->pitch;
            osimg2->roll = ospr2->roll;
            img = osimg2;
            break;
        }

        default:
        {
            OculusSonarImage *osimg = new OculusSonarImage();
            osimg->pingStartTime = ospr->pingStartTime;
            osimg->sonarFrequency = ospr->frequency;
            this->operatingFrequency = ospr->frequency;
            osimg->temperature = ospr->temperature;
            this->temperature = ospr->temperature;
            osimg->pressure = ospr->pressure;
            this->pressure = ospr->pressure;
            img = osimg;
            break;
        }
        }
        img->imageWidth = beams;
        img->imageHeight = ranges;
        img->data = new uint8_t[imageSize];
        memcpy(img->data, startAddress + imageOffset, imageSize);

        // Set last image pointer to new image and delete old one
        delete lastImage;
        lastImage = img;

        // New image ready, notify all callbacks
        SonarCallback cb;
        std::lock_guard<std::mutex> lock(*callbackMutex);
        for (uint16_t i = 0; i < callbacks->size(); i++)
        {
            cb = callbacks->at(i);
            cb(this, img);
        }
    }
    else
    {
        // Message format not correct
    }
}

const char *OculusSonar::getLocation()
{
    return sonarAddress;
}

double OculusSonar::getOperatingFrequency()
{
    return operatingFrequency;
}

uint16_t OculusSonar::getBeamCount()
{
    return beams;
}

double OculusSonar::getBeamSeparation()
{
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
        case OculusPartNumberType::partNumberM370s_Artemis:
        case OculusPartNumberType::partNumberM370s_Deep:
            return 0.5;
        case OculusPartNumberType::partNumberM750d:
        case OculusPartNumberType::partNumberM750d_Artemis:
        case OculusPartNumberType::partNumberM750d_Fusion:
            return 0.25;
        case OculusPartNumberType::partNumberM1200d:
        case OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusPartNumberType::partNumberM1200d_Deep:
            {
                if (fireMode == 1) // Low frequency
                    return 0.25;
                else if (fireMode == 2) // High frequency
                    return 0.16;
                break; // Unknown fire mode -> Cannot determine beam separation for M1200d
            }
        default:
            break; // Unknown part number -> Cannot determine beam separation
    }
    return 0.0;
}

double OculusSonar::getMinimumRange()
{
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
        case OculusPartNumberType::partNumberM370s_Artemis:
        case OculusPartNumberType::partNumberM370s_Deep:
            return 0.2;
        case OculusPartNumberType::partNumberM750d:
        case OculusPartNumberType::partNumberM750d_Artemis:
        case OculusPartNumberType::partNumberM750d_Fusion:
            return 0.1;
        case OculusPartNumberType::partNumberM1200d:
        case OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusPartNumberType::partNumberM1200d_Deep:
            return 0.1;
        default:
            break; // Unknown part number -> Cannot determine minimum range
    }
    return 0.0;
}

double OculusSonar::getMaximumRange()
{
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
        case OculusPartNumberType::partNumberM370s_Artemis:
        case OculusPartNumberType::partNumberM370s_Deep:
            return 200.0;
        case OculusPartNumberType::partNumberM750d:
        case OculusPartNumberType::partNumberM750d_Artemis:
        case OculusPartNumberType::partNumberM750d_Fusion:
            {
                if (fireMode == 1) // Low frequency
                    return 120.0;
                else if (fireMode == 2) // High frequency
                    return 40.0;
                break; // Unknown fire mode -> Cannot determine maximum range for M750d
            }
        case OculusPartNumberType::partNumberM1200d:
        case OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusPartNumberType::partNumberM1200d_Deep:
            {
                if (fireMode == 1) // Low frequency
                    return 40.0;
                else if (fireMode == 2) // High frequency
                    return 10.0;
                break; // Unknown fire mode -> Cannot determine maximum range for M1200d
            }
        default:
            break; // Unknown part number -> Cannot determine maximum range
    }
    return 0.0;
}

double OculusSonar::getRangeResolution()
{
    return rangeResolution;
}

uint32_t OculusSonar::getRangeBinCount()
{
    return rangeBinCount;
}

double OculusSonar::getHorzFOV()
{
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
        case OculusPartNumberType::partNumberM370s_Artemis:
        case OculusPartNumberType::partNumberM370s_Deep:
        case OculusPartNumberType::partNumberM750d:
        case OculusPartNumberType::partNumberM750d_Artemis:
        case OculusPartNumberType::partNumberM750d_Fusion:
            return 130.0;
        case OculusPartNumberType::partNumberM1200d:
        case OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusPartNumberType::partNumberM1200d_Deep:
            {
                if (fireMode == 1) // Low frequency
                    return 130.0;
                else if (fireMode == 2) // High frequency
                    return 60.0;
                break; // Unknown fire mode -> Cannot determine horzFOV for M1200d
            }
        default:
            break; // Unknown part number -> Cannot determine horzFOV
    }
    return 0.0;
}

double OculusSonar::getVertFOV()
{
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
        case OculusPartNumberType::partNumberM370s_Artemis:
        case OculusPartNumberType::partNumberM370s_Deep:
        case OculusPartNumberType::partNumberM750d:
        case OculusPartNumberType::partNumberM750d_Artemis:
        case OculusPartNumberType::partNumberM750d_Fusion:
            return 20.0;
        case OculusPartNumberType::partNumberM1200d:
        case OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusPartNumberType::partNumberM1200d_Deep:
            {
                if (fireMode == 1) // Low frequency
                    return 20.0;
                else if (fireMode == 2) // High frequency
                    return 12.0;
                break; // Unknown fire mode -> Cannot determine vertFOV for M1200d
            }
        default:
            break; // Unknown part number -> Cannot determine vertFOV
    }
    return 0.0;
}

double OculusSonar::getAngularResolution()
{
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
        case OculusPartNumberType::partNumberM370s_Artemis:
        case OculusPartNumberType::partNumberM370s_Deep:
            return 0.2;
        case OculusPartNumberType::partNumberM750d:
        case OculusPartNumberType::partNumberM750d_Artemis:
        case OculusPartNumberType::partNumberM750d_Fusion:
            {
                if (fireMode == 1) // Low frequency
                    return 1.0;
                else if (fireMode == 2) // High frequency
                    return 0.6;
                break; // Unknown fire mode -> Cannot determine angular resolution for M750d
            }
        case OculusPartNumberType::partNumberM1200d:
        case OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusPartNumberType::partNumberM1200d_Deep:
            {
                if (fireMode == 1) // Low frequency
                    return 0.6;
                else if (fireMode == 2) // High frequency
                    return 0.4;
                break; // Unknown fire mode -> Cannot determine angular resolution for M1200d
            }
        default:
            break; // Unknown part number -> Cannot determine angular resolution
    }
    return 0.0;
}

std::string OculusSonar::getDeviceName() {
    switch (partNumber)
    {
        case OculusPartNumberType::partNumberM370s:
            return "Oculus M370s";
        case OculusPartNumberType::partNumberM370s_Artemis:
            return "Oculus M370s-Artemis";
        case OculusPartNumberType::partNumberM370s_Deep:
            return "Oculus M370s-Deep";
        case OculusPartNumberType::partNumberM750d:
            return "Oculus M750d";
        case OculusPartNumberType::partNumberM750d_Artemis:
            return "Oculus M750d-Artemis";
        case OculusPartNumberType::partNumberM750d_Fusion:
            return "Oculus M750d-Fusion";
        case OculusPartNumberType::partNumberM1200d:
            return "Oculus M1200d";
        case OculusPartNumberType::partNumberM1200d_Artemis:
            return "Oculus M1200d-Artemis";
        case OculusPartNumberType::partNumberM1200d_Deep:
            return "Oculus M1200d-Deep";
        default:
            return "Unknown Oculus Sonar";
    }
}

OculusM1200d::OculusM1200d() : OculusSonar()
{
}

OculusM1200d::~OculusM1200d()
{
}

SonarImage::SonarImage()
{
    imageType = SonarImageType::SonarImageObject;
    imageHeight = 0;
    imageWidth = 0;
    data = nullptr;
}

SonarImage::~SonarImage()
{
    if (data != nullptr)
    {
        delete[] data;
        data = nullptr;
    }
}

OculusSonarImage::OculusSonarImage() : SonarImage()
{
    imageType = SonarImageType::OculusSonarImageObject;
    pingStartTime = 0;
    sonarFrequency = 0;
    temperature = 0;
    pressure = 0;
}

OculusSonarImage::~OculusSonarImage()
{
}

OculusSonarImage2::OculusSonarImage2() : SonarImage()
{
    imageType = SonarImageType::OculusSonarImage2Object;
    pingStartTime = 0;
    sonarFrequency = 0;
    temperature = 0;
    pressure = 0;
    heading = 0;
    pitch = 0;
    roll = 0;
}

OculusSonarImage2::~OculusSonarImage2()
{
}