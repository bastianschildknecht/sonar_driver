// File: ezsocket.cxx

#include <ezsocket.hxx>

#ifdef _WIN32

#include <windows.h>
#include <winsock2.h>
#include <WS2tcpip.h>

#else

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>

#endif

using namespace EZSocket;

Socket::Socket()
{
    state = SocketState::Exists;

#ifdef _WIN32
    int err = WSAStartup(MAKEWORD(2,0), &wsadata);
    if (err != 0)
    {
        state = SocketState::StartupError;
    }
#else
    socket_fd = 0;
#endif
}

Socket::~Socket()
{
#ifdef _WIN32
    if (state != SocketState::StartupError)
    {
        if (state != SocketState::InitError)
        {
            closesocket(socket_fd);
        }
        WSACleanup();
    }
#else
    if (state != SocketState::InitError)
    {
        close(socket_fd);
    }
#endif
}

void Socket::connectToHost(const char *hostname, uint16_t port)
{
    if (state == SocketState::Ready)
    {
        host_addr.sin_family = AF_INET;
        host_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, hostname, &host_addr.sin_addr) <= 0)
        {
            state = SocketState::AddressError;
            return;
        }

        if (connect(socket_fd, (struct sockaddr *)&host_addr, sizeof(host_addr)) < 0)
        {
            state = SocketState::ConnectError;
            return;
        }

        state = SocketState::Connected;
    }
}

void Socket::disconnect()
{
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        closesocket(socket_fd);
#else
        close(socket_fd);
#endif
        initSocket();
    }
}

int32_t Socket::readData(void *buffer, int32_t maxLength)
{
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        return recv(socket_fd, (char *) buffer, maxLength, 0);
#else
        return read(socket_fd, buffer, (size_t)maxLength);
#endif
    }
    else
    {
        return -1;
    }
}

int32_t Socket::writeData(const void *buffer, int32_t length)
{
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        return send(socket_fd, (const char *)buffer, length, 0);
#else
        return write(socket_fd, buffer, (size_t)length);
#endif
    }
    else
    {
        return -1;
    }
}

void Socket::setReceiveBufferSize(int32_t size)
{
    if (state == SocketState::Ready)
    {
        size = size >= 1024 ? size : 1024;
#ifdef _WIN32
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, (char*)&size, sizeof(size));
#else
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
#endif
    }
}

uint64_t Socket::bytesAvailable()
{
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        u_long value = 0;
        ioctlsocket(socket_fd, FIONREAD, &value);
        return value;
#else
        int32_t value = 0;
        ioctl(socket_fd, FIONREAD, &value);
        return value
#endif
    }
    else
    {
        return 0;
    }
}

SocketState Socket::getState()
{
    return state;
}

TCPSocket::TCPSocket() : Socket()
{
    TCPSocket::initSocket();
}

TCPSocket::~TCPSocket()
{
}

void TCPSocket::initSocket()
{
#ifdef _WIN32
    if (state != SocketState::StartupError)
    {
        socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd == INVALID_SOCKET)
        {
            state = SocketState::InitError;
        }
        else
        {
            state = SocketState::Ready;
        }
    }
#else
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0)
    {
        state = SocketState::InitError;
    }
    else
    {
        state = SocketState::Ready;
    }
#endif
}

UDPSocket::UDPSocket() : Socket()
{
    UDPSocket::initSocket();
}

UDPSocket::~UDPSocket()
{
}

void UDPSocket::initSocket()
{
#ifdef _WIN32
    if (state != SocketState::StartupError)
    {
        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd == INVALID_SOCKET)
        {
            state = SocketState::InitError;
        }
        else
        {
            state = SocketState::Ready;
        }
    }
#else
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        state = SocketState::InitError;
    }
    else
    {
        state = SocketState::Ready;
    }
#endif
}