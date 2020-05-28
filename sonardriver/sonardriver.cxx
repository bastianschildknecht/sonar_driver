#include <iostream>
#include <cstdlib>
#include <ezsocket.hxx>

using namespace std;
using namespace EZSocket;

int main(int argc, char *argv[])
{
    int bufferSize = 1<<12;
    
    Socket *socket = new TCPSocket();
    if (socket->getState() == SocketState::Ready)
    {
        socket->setReceiveBufferSize(2*bufferSize);
        socket->connectToHost("192.168.1.10", 52100);
        if (socket->getState() == SocketState::Connected)
        {
            char *buff = new char[bufferSize];
            int bytesRead = socket->readData(buff,bufferSize);
            cout << "Received " << bytesRead << " bytes" << endl;
            cout << "Data: " << buff << endl;
            delete[] buff;
            buff = nullptr;
        }
        else
        {
            cout << "Could not connect!" << endl;
        }
    }
    else
    {
        cout << "Initialization error!" << endl;
    }
    delete socket;
    socket = nullptr;
    exit(EXIT_SUCCESS);
}