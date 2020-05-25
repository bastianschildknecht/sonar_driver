// File: ezsocket.cxx

#include <ezsocket.hxx>

#ifdef _WIN32

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
    socket_fd = 0;
}

Socket::~Socket()
{
}

void Socket::connectToHost(const char *hostname, uint16_t port)
{
    if (state == SocketState::Ready)
    {
#ifdef _WIN32
#else
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
#endif
        state = SocketState::Connected;
    }
}

void Socket::disconnect()
{
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
#else
        close(socket_fd);
        initSocket();
#endif
    }
}

int32_t Socket::readData(void *buffer, int32_t maxLength)
{
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        return 0;
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
        return 0;
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
#else
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
#endif
    }
}

uint64_t Socket::bytesAvailable()
{
    int32_t value = 0;
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
#else
        ioctl(socket_fd, FIONREAD, &value);
#endif
    }
    return value;
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
#ifdef _WIN32
#else
    if (state != SocketState::InitError)
    {
        close(socket_fd);
    }
#endif
}

void TCPSocket::initSocket()
{
#ifdef _WIN32
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
#ifdef _WIN32
#else
    if (state != SocketState::InitError)
    {
        close(socket_fd);
    }
#endif
}

void UDPSocket::initSocket()
{
#ifdef _WIN32
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