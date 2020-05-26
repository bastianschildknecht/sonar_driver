#include <iostream>
#include <cstdlib>
#include <ezsocket.hxx>

using namespace std;
using namespace EZSocket;

int main(int argc, char *argv[])
{
    int bufferSize = 1<<16;
    cout << "Buffer Size: " << bufferSize << endl;
    Socket *socket = new TCPSocket();
    if (socket->getState() == SocketState::Ready)
    {
        socket->setReceiveBufferSize(2*bufferSize);
        socket->connectToHost("216.58.215.227", 80);
        if (socket->getState() == SocketState::Connected)
        {
            // Send GET
            char *message = "GET / HTTP/1.1\r\n\r\n";
            int bytesSent = socket->writeData(message, strlen(message));
            cout << "Sent " << bytesSent << " bytes" << endl;
            Sleep(1000);
            // Receive
            char *buff = new char[bufferSize];
            cout << "Available bytes: " << socket->bytesAvailable() << endl;
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