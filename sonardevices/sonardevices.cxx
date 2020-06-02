// File: sonardevices.cxx

#include <iostream>
#include <vector>
#include <ezsocket.hxx>
#include <sonardevices.hxx>

using namespace EZSocket;
using namespace SonarDevices;

Sonar::Sonar()
{
    state = SonarState::Ready;
    callbacks = new std::vector<void (*)(SonarImage *)>();
}

Sonar::~Sonar()
{
    delete callbacks;
    callbacks = nullptr;
}

void Sonar::registerCallback(void (*callback)(SonarImage *sonarImage))
{
    callbacks->push_back(callback);
}

SonarState Sonar::getState()
{
    return state;
}

OculusSonar::OculusSonar() : Sonar()
{
    sonarTCPSocket = new TCPSocket();
    sonarUDPSocket = new UDPSocket();
    sonarAddress = new char[STR_ADDRESS_LEN];
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
                connect(sonarAddress);
            }
            delete[] buf;
            buf = nullptr;
        }
    }
}

void OculusSonar::connect(const char *address)
{
    if (sonarTCPSocket->getState() == SocketState::Ready)
    {
        sonarTCPSocket->connectToHost(address, OCULUS_TCP_PORT);
        if (sonarTCPSocket->getState() == SocketState::Connected)
        {
            strncpy(sonarAddress, address, STR_ADDRESS_LEN);
            state = SonarState::Connected;
        }
    }
}

void OculusSonar::disconnect()
{
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

void OculusSonar::fire()
{
}

const char *OculusSonar::getLocation()
{
    return sonarAddress;
}

SonarImage *OculusSonar::getLastImage()
{
    return NULL;
}

OculusM1200d::OculusM1200d() : OculusSonar()
{
}

OculusM1200d::~OculusM1200d()
{
}

SonarImage::SonarImage()
{
}

SonarImage::~SonarImage()
{
}