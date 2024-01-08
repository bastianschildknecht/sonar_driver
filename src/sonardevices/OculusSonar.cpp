
#include <sonar_driver/sonardevices/OculusSonar.h>


OculusSonar::OculusSonar() : Sonar(){
    partNumber = OculusMessages::OculusPartNumberType::partNumberUndefined;
    sonarTCPSocket = new EZSocket::TCPSocket();
    sonarUDPSocket = new EZSocket::UDPSocket();
    sonarAddress = new char[STR_ADDRESS_LEN];
    oculusPingRate = OculusMessages::PingRateType::pingRateStandby;
    operatingFrequency = 0.0;
    beams = 0;
    rangeBinCount = 0;
    rangeResolution = 0.0;
}

OculusSonar::~OculusSonar(){
    delete sonarTCPSocket;
    sonarTCPSocket = nullptr;
    delete sonarUDPSocket;
    sonarUDPSocket = nullptr;
    delete[] sonarAddress;
    sonarAddress = nullptr;
}

void OculusSonar::findAndConnect(){
    if (sonarUDPSocket->getState() == EZSocket::SocketState::Ready)
    {
        sonarUDPSocket->bindToAddress("ANY", OCULUS_UDP_PORT);
        if (sonarUDPSocket->getState() == EZSocket::SocketState::Bound)
        {
            char *buf = new char[256];
            if (sonarUDPSocket->waitForDataAndAddress(buf, 256, sonarAddress, STR_ADDRESS_LEN) > 0)
            {
                this->partNumber = determinePartNumber(buf);
                this->connect(sonarAddress);
            }
            delete[] buf;
            buf = nullptr;
        }
    }
}

OculusMessages::OculusPartNumberType OculusSonar::determinePartNumber(char *udpBroadcastMessage){
    // First check if the message is a valid Oculus message
    // The first two bytes should containt the Oculus Check ID
    uint16_t checkID = *(uint16_t *)udpBroadcastMessage;
    if (checkID != OCULUS_CHECK_ID)
    {
        return OculusMessages::OculusPartNumberType::partNumberUndefined;
    }

    // The UDP broadcast message contains a OculusMessageHeader as
    // the first part of the message.
    OculusMessages::OculusMessageHeader *oculusMessageHeader = (OculusMessages::OculusMessageHeader *)udpBroadcastMessage;

    // The spare2 field contains the part number.
    uint16_t numericPartNumber = oculusMessageHeader->spare2;

    // Cast the short to a OculusPartNumberType
    OculusMessages::OculusPartNumberType partNumber = static_cast<OculusMessages::OculusPartNumberType>(numericPartNumber);

    return partNumber;
}

void OculusSonar::connect(const char *address){
    if (sonarTCPSocket->getState() == EZSocket::SocketState::Ready)
    {
        sonarTCPSocket->setReceiveBufferSize(200000);
        sonarTCPSocket->connectToHost(address, OCULUS_TCP_PORT);
        if (sonarTCPSocket->getState() == EZSocket::SocketState::Connected)
        {
            strncpy(sonarAddress, address, STR_ADDRESS_LEN);
            callbackThreadActive = true;
            callbackThread = new std::thread([this] { this->invokeCallbacks(); });
            callbackThreadStarted = true;
            state = SonarState::Connected;
        }
    }
}

void OculusSonar::disconnect(){
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
    if (sonarUDPSocket->getState() == EZSocket::SocketState::Ready && sonarTCPSocket->getState() == EZSocket::SocketState::Ready)
    {
        state = SonarState::Ready;
    }
    else
    {
        state = SonarState::SocketError;
    }
}

void OculusSonar::configure(int mode, double range, double gain, double speedOfSound, double salinity, bool gainAssist, uint8_t gamma, uint8_t netSpeedLimit){
    fireMode = mode;
    currRange = range;
    currGain = gain;
    this->speedOfSound = speedOfSound;
    this->salinity = salinity;
    gainAssistActive = gainAssist;
    this->gamma = gamma;
    this->netSpeedLimit = netSpeedLimit;
}

uint8_t OculusSonar::setPingRate(uint8_t frequency){
    if (frequency == 0)
    {
        pingRate = 0;
        oculusPingRate = OculusMessages::PingRateType::pingRateStandby;
    }
    else if (frequency <= 2)
    {
        pingRate = 2;
        oculusPingRate = OculusMessages::PingRateType::pingRateLowest;
    }
    else if (frequency <= 5)
    {
        pingRate = 5;
        oculusPingRate = OculusMessages::PingRateType::pingRateLow;
    }
    else if (frequency <= 10)
    {
        pingRate = 10;
        oculusPingRate = OculusMessages::PingRateType::pingRateNormal;
    }
    else if (frequency <= 15)
    {
        pingRate = 15;
        oculusPingRate = OculusMessages::PingRateType::pingRateHigh;
    }
    else
    {
        pingRate = 40;
        oculusPingRate = OculusMessages::PingRateType::pingRateHighest;
    }
    return pingRate;
}

void OculusSonar::fire(){
    OculusMessages::OculusSimpleFireMessage sfm;
    memset(&sfm, 0, sizeof(OculusMessages::OculusSimpleFireMessage));

    sfm.head.msgId = OculusMessages::OculusMessageType::messageSimpleFire;
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

    sonarTCPSocket->writeData(&sfm, sizeof(OculusMessages::OculusSimpleFireMessage));
}


/// @brief Central function processing all new available bytes on the thread it was created on
void OculusSonar::invokeCallbacks(){

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
        if (bytesAvailable <= 0){
            continue;
        }

        // Check if buffer is big enough and expand if required
        if (bytesAvailable > rxBuffSize - rxBuffPos){
            rxBuffSize = bytesAvailable + rxBuffPos;
            rxBuff = (uint8_t *)realloc(rxBuff, rxBuffSize);
        }

        bytesRead = sonarTCPSocket->readData(&rxBuff[rxBuffPos], bytesAvailable);
        // Check for socket errors or disconnects
        if (bytesRead < 0){
            break;
        }
        rxBuffPos += bytesRead;

        // Process received data
        currPktSize = sizeof(OculusMessages::OculusMessageHeader);
        if (rxBuffPos < currPktSize){
            continue;
        }
        OculusMessages::OculusMessageHeader *omh = (OculusMessages::OculusMessageHeader *)rxBuff;

        // Check if data is valid via oculus ID
        if (omh->oculusId != OCULUS_CHECK_ID)
        {
            // Data invalid, flush buffer
            rxBuffPos = 0;
            continue;
        }

        currPktSize += omh->payloadSize;

        // Check if payload is also in the buffer and process message
        if (rxBuffPos < currPktSize){
            continue;
        }

        // Check which message type was received
        switch (omh->msgId){
            case OculusMessages::OculusMessageType::messageSimplePingResult:
                // Received result of ping
                processSimplePingResult((OculusMessages::OculusSimplePingResult *)omh);
                break;
            case OculusMessages::OculusMessageType::messageUserConfig:
                // Received user config
                break;
            case OculusMessages::OculusMessageType::messageDummy:
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

    // Free memory
    free(rxBuff);
    rxBuff = nullptr;
}

void OculusSonar::processSimplePingResult(OculusMessages::OculusSimplePingResult *ospr){
    uint8_t *startAddress = (uint8_t *)ospr;

    uint32_t imageSize;
    uint32_t imageOffset;
    uint16_t beams;
    uint16_t ranges;

    uint16_t version = ospr->fireMessage.head.msgVersion;
    
    OculusMessages::OculusSimplePingResult2 *ospr2 = (OculusMessages::OculusSimplePingResult2 *)ospr;

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
    if (ospr->fireMessage.head.payloadSize + sizeof(OculusMessages::OculusMessageHeader) == imageOffset + imageSize){
        // Copy image into sonar image
        SonarImage *img;

        switch (version){
            case 2:{
                this->operatingFrequency = ospr2->frequency;
                this->temperature = ospr2->temperature;
                this->pressure = ospr2->pressure;
                break;
            }
            default:{
                this->operatingFrequency = ospr->frequency;
                this->temperature = ospr->temperature;
                this->pressure = ospr->pressure;
                break;
            }
        }

        memcpy(lastImage->bearingTable.get(), startAddress + 122, beams * sizeof(int16_t));

        memcpy(lastImage->data.get(), startAddress + imageOffset, imageSize);


        // New image ready, notify all callbacks
        SonarCallback cb;
        std::lock_guard<std::mutex> lock(*callbackMutex);
        for (uint16_t i = 0; i < callbacks->size(); i++)
        {
            cb = callbacks->at(i);
            cb(this, lastImage);
        }
    }
    else
    {
        // Message format not correct
    }
}


std::vector<int16_t> OculusSonar::getBearingTable(){
    printf("sonardevices.cpp getBearingTable(): Starting transform\n");
    int n = lastImage->imageWidth;
    std::vector<int16_t> bearingVector(n);  // preallocate space for the bearings
    std::transform(lastImage->bearingTable.get(), lastImage->bearingTable.get() + n, bearingVector.begin(), [](int16_t bearing) {
        return bearing;
    });
    printf("sonardevices.cpp getBearingTable(): Finished transform\n");
    return bearingVector;
}

const char *OculusSonar::getLocation(){
    return sonarAddress;
}

double OculusSonar::getOperatingFrequency(){
    return operatingFrequency;
}

uint16_t OculusSonar::getBeamCount(){
    return beams;
}

double OculusSonar::getBeamSeparation(){
    switch (partNumber)
    {
        case OculusMessages::OculusPartNumberType::partNumberM370s:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
            return 0.5;
        case OculusMessages::OculusPartNumberType::partNumberM750d:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            return 0.25;
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
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

double OculusSonar::getMinimumRange(){
    switch (partNumber)
    {
        case OculusMessages::OculusPartNumberType::partNumberM370s:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
            return 0.2;
        case OculusMessages::OculusPartNumberType::partNumberM750d:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            return 0.1;
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
            return 0.1;
        default:
            break; // Unknown part number -> Cannot determine minimum range
    }
    return 0.0;
}

double OculusSonar::getMaximumRange(){
    switch (partNumber)
    {
        case OculusMessages::OculusPartNumberType::partNumberM370s:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
            return 200.0;
        case OculusMessages::OculusPartNumberType::partNumberM750d:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            {
                if (fireMode == 1) // Low frequency
                    return 120.0;
                else if (fireMode == 2) // High frequency
                    return 40.0;
                break; // Unknown fire mode -> Cannot determine maximum range for M750d
            }
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
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

double OculusSonar::getRangeResolution(){
    return rangeResolution;
}

uint32_t OculusSonar::getRangeBinCount(){
    return rangeBinCount;
}

double OculusSonar::getHorzFOV(){
    switch (partNumber)
    {
        case OculusMessages::OculusPartNumberType::partNumberM370s:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
        case OculusMessages::OculusPartNumberType::partNumberM750d:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            return 130.0;
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
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

double OculusSonar::getVertFOV(){
    switch (partNumber)
    {
        case OculusMessages::OculusPartNumberType::partNumberM370s:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
        case OculusMessages::OculusPartNumberType::partNumberM750d:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            return 20.0;
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
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

double OculusSonar::getAngularResolution(){
    switch (partNumber)
    {
        case OculusMessages::OculusPartNumberType::partNumberM370s:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
            return 0.2;
        case OculusMessages::OculusPartNumberType::partNumberM750d:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            {
                if (fireMode == 1) // Low frequency
                    return 1.0;
                else if (fireMode == 2) // High frequency
                    return 0.6;
                break; // Unknown fire mode -> Cannot determine angular resolution for M750d
            }
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
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
        case OculusMessages::OculusPartNumberType::partNumberM370s:
            return "Oculus M370s";
        case OculusMessages::OculusPartNumberType::partNumberM370s_Artemis:
            return "Oculus M370s-Artemis";
        case OculusMessages::OculusPartNumberType::partNumberM370s_Deep:
            return "Oculus M370s-Deep";
        case OculusMessages::OculusPartNumberType::partNumberM750d:
            return "Oculus M750d";
        case OculusMessages::OculusPartNumberType::partNumberM750d_Artemis:
            return "Oculus M750d-Artemis";
        case OculusMessages::OculusPartNumberType::partNumberM750d_Fusion:
            return "Oculus M750d-Fusion";
        case OculusMessages::OculusPartNumberType::partNumberM1200d:
            return "Oculus M1200d";
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Artemis:
            return "Oculus M1200d-Artemis";
        case OculusMessages::OculusPartNumberType::partNumberM1200d_Deep:
            return "Oculus M1200d-Deep";
        default:
            return "Unknown Oculus Sonar";
    }
}
