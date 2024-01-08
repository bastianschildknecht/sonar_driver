
#include <vector>
#include <cstring>
#include <memory.h>
#include <sonar_driver/ezsocket/ezsocket.hxx>
#include <sonar_driver/sonardevices/Sonar.h>
#include <sonar_driver/sonardevices/OculusMessages.h>

#include <iostream>

using namespace EZSocket;

Sonar::Sonar(){
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

Sonar::~Sonar(){
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

void Sonar::registerCallback(SonarCallback callback){
    std::lock_guard<std::mutex> lock(*callbackMutex);
    callbacks->push_back(callback);
}

SonarState Sonar::getState(){
    return state;
}

SonarImage Sonar::getLastImage(){
    return *lastImage;
}

uint8_t Sonar::getFireMode(){
    return fireMode;
}

uint8_t Sonar::getPingRate(){
    return pingRate;
}

double Sonar::getCurrentRange(){
    return currRange;
}

double Sonar::getCurrentGain(){
    return currGain;
}

bool Sonar::gainAssistEnabled(){
    return gainAssistActive;
}

uint8_t Sonar::getGamma(){
    return gamma;
}

double Sonar::getSpeedOfSound(){
    return speedOfSound;
}

double Sonar::getSalinity(){
    return salinity;
}

double Sonar::getTemperature(){
    return temperature;
}

double Sonar::getPressure(){
    return pressure;
}

uint8_t Sonar::getNetworkSpeedLimit(){
    return netSpeedLimit;
}


