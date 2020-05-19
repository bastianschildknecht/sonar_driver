#include <iostream>
#include <cstdlib>
#include <ezsocket.hxx>

using namespace std;
using namespace EZSocket;

int main(int argc, char *argv[])
{
    Socket *socket = new TCPSocket();
    delete socket;
    socket = nullptr;
    exit(EXIT_SUCCESS);
}