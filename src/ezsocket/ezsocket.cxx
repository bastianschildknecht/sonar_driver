// File: ezsocket.cxx

#include <sonar_driver/ezsocket/ezsocket.hxx>
#include <iostream>
#include <mutex>
#include <cstring>

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
    mutex = new std::mutex();

#ifdef _WIN32
    int err = WSAStartup(MAKEWORD(2, 0), &wsadata);
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
    delete mutex;
    mutex = nullptr;

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

void Socket::connectToHost(const std::string& address, uint16_t port)
{
    mutex->lock();
    if (state == SocketState::Ready)
    {
        host_addr.sin_family = AF_INET;
        host_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, address.c_str(), &host_addr.sin_addr) <= 0)
        {
            state = SocketState::AddressError;
            mutex->unlock();
            return;
        }

        if (connect(socket_fd, (struct sockaddr *)&host_addr, sizeof(host_addr)) < 0)
        {
            state = SocketState::ConnectError;
            mutex->unlock();
            return;
        }

        state = SocketState::Connected;
    }
    mutex->unlock();
}

void Socket::disconnect()
{
    mutex->lock();
    if (state == SocketState::Connected || state == SocketState::Bound)
    {
#ifdef _WIN32
        closesocket(socket_fd);
#else
        close(socket_fd);
#endif
        initSocket();
    }
    mutex->unlock();
}

void Socket::bindToAddress(const std::string& address, uint16_t port)
{
    mutex->lock();
    if (state == SocketState::Ready)
    {
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);

        // Load address
        if (strncmp(address.c_str(), "ANY", 16) == 0)
        {
#ifdef _WIN32
            server_addr.sin_addr.S_un.S_addr = INADDR_ANY;
#else
            server_addr.sin_addr.s_addr = INADDR_ANY;
#endif
        }
        else
        {
            if (inet_pton(AF_INET, address.c_str(), &server_addr.sin_addr) <= 0)
            {
                state = SocketState::AddressError;
                mutex->unlock();
                return;
            }
        }

        // Bind socket
        if (bind(socket_fd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            state = SocketState::BindError;
            mutex->unlock();
            return;
        }
        state = SocketState::Bound;
    }
    mutex->unlock();
}

int32_t Socket::readData(void *buffer, int32_t maxLength)
{
    mutex->lock();
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        int32_t bytesRead = recv(socket_fd, (char *)buffer, maxLength, 0);
#else
        int32_t bytesRead = recv(socket_fd, buffer, (size_t)maxLength, 0);
#endif
        mutex->unlock();
        return bytesRead;
    }
    else
    {
        mutex->unlock();
        return -1;
    }
}

int32_t Socket::waitForDataAndAddress(void *buffer, int32_t maxLength, std::string& addressBuffer, size_t bufferLength)
{
    mutex->lock();
    if (state == SocketState::Bound)
    {
        struct sockaddr_in src_addr;
        int src_addr_len = sizeof(src_addr);
#ifdef _WIN32
        int32_t ret = recvfrom(socket_fd, (char *)buffer, maxLength, 0, (struct sockaddr *)&src_addr, &src_addr_len);
#else
        int32_t ret = recvfrom(socket_fd, buffer, (size_t)maxLength, 0, (struct sockaddr *)&src_addr, (socklen_t *)&src_addr_len);
#endif

        if (bufferLength >= INET_ADDRSTRLEN)
        {
            char* addressBufferPtr = &addressBuffer[0];
            inet_ntop(AF_INET, &src_addr.sin_addr, addressBufferPtr, bufferLength);
        }


        mutex->unlock();
        return ret;
    }
    else
    {
        mutex->unlock();
        return -1;
    }
}

int32_t Socket::writeData(const void *buffer, int32_t length)
{
    mutex->lock();
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        int32_t bytesSent = send(socket_fd, (const char *)buffer, length, 0);
#else
        int32_t bytesSent = send(socket_fd, buffer, (size_t)length, 0);
#endif
        mutex->unlock();
        return bytesSent;
    }
    else
    {
        mutex->unlock();
        return -1;
    }
}

void Socket::setReceiveBufferSize(int32_t size)
{
    mutex->lock();
    if (state == SocketState::Ready)
    {
        size = size >= 1024 ? size : 1024;
#ifdef _WIN32
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, (char *)&size, sizeof(size));
#else
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
#endif
    }
    mutex->unlock();
}

uint64_t Socket::bytesAvailable()
{
    mutex->lock();
    if (state == SocketState::Connected)
    {
#ifdef _WIN32
        u_long value = 0;
        ioctlsocket(socket_fd, FIONREAD, &value);
#else
        int32_t value = 0;
        ioctl(socket_fd, FIONREAD, &value);
#endif
        mutex->unlock();
        return value;
    }
    else
    {
        mutex->unlock();
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