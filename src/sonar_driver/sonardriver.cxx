#include <iostream>
#include <cstdlib>
#include <sonar_driver/sonardevices/sonardevices.hxx>

using namespace std;
using namespace SonarDevices;

void callback(SonarImage *img)
{
    cout << "Received Sonar Image!" << endl;
    cout << "\tImage width: " << img->imageWidth << endl;
    cout << "\tImage height: " << img->imageHeight << endl;
    cout << "\tImage type: " << img->imageType << endl;
    switch (img->imageType)
    {
    case SonarImageType::OculusSonarImageObject:
    {
        OculusSonarImage *osi = (OculusSonarImage *) img;
        cout << "\tPing start time: " << osi->pingStartTime << endl;
        cout << "\tSonar frequency: " << osi->sonarFrequency << endl;
        cout << "\tTemperature: " << osi->temperature << endl;
        cout << "\tPressure: " << osi->pressure << endl;
        break;
    }
    case SonarImageType::OculusSonarImage2Object:
    {
        OculusSonarImage2 *osi2 = (OculusSonarImage2 *) img;
        cout << "\tPing start time: " << osi2->pingStartTime << endl;
        cout << "\tSonar frequency: " << osi2->sonarFrequency << endl;
        cout << "\tTemperature: " << osi2->temperature << endl;
        cout << "\tPressure: " << osi2->pressure << endl;
        cout << "\tHeading: " << osi2->heading << endl;
        cout << "\tPitch: " << osi2->pitch << endl;
        cout << "\tRoll: " << osi2->roll << endl;
        break;
    }
    default:
        break;
    }
}

int main(int argc, char *argv[])
{
    Sonar *sonar = new OculusSonar();
    sonar->registerCallback(callback);
    sonar->configure(2, 5, 0.5, 0, 0, false, 1, 255);
    sonar->setPingRate(40);
    cout << "Looking for sonar..." << endl;
    sonar->findAndConnect();
    if (sonar->getState() == SonarState::Connected)
    {
        cout << "Found sonar at location: " << sonar->getLocation() << endl;
        sonar->fire();
        Sleep(30000);
    }
    sonar->disconnect();
    delete sonar;
    sonar = nullptr;
    exit(EXIT_SUCCESS);
}