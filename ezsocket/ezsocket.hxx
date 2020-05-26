// File: ezsocket.hxx

#ifndef EZSOCKET_HXX
#define EZSOCKET_HXX

#include <cstdint>

#ifdef _WIN32

#include <winsock2.h>

#else

#include <netinet/in.h>

#endif

namespace EZSocket
{

    enum SocketState
    {
        Exists,
        Ready,
        Connected,
        InitError,
        ConnectError,
        AddressError,
        StartupError
    };

    class Socket
    {
    public:
        Socket();
        virtual ~Socket() = 0;

        // Connects the socket to the specified host on the specified port
        // (blocking)
        virtual void connectToHost(const char *hostname, uint16_t port);

        // Disconnects the socket from the host
        // (Only needed if you want to reconnect or change receive buffer size)
        virtual void disconnect();

        // Reads available bytes from the socket into a buffer up to a maximum length
        virtual int32_t readData(void *buffer, int32_t maxLength);

        // Writes a specified amount of bytes to the socket
        virtual int32_t writeData(const void *buffer, int32_t length);

        // Changes the size of the socket receive buffer
        // (Must be used before establishing a connection)
        virtual void setReceiveBufferSize(int32_t size);

        // Returns the number of received bytes on the socket
        virtual uint64_t bytesAvailable();

        // Get the current state the socket is in (includes errors)
        virtual SocketState getState();

    protected:
        SocketState state;
        struct sockaddr_in host_addr;
#ifdef _WIN32
        WSAData wsadata;
        SOCKET socket_fd;
#else
        int socket_fd;
#endif
        virtual void initSocket() = 0;
    };

    class TCPSocket : public Socket
    {
    public:
        TCPSocket();
        ~TCPSocket();

    protected:
        void initSocket();
    };

    class UDPSocket : public Socket
    {
    public:
        UDPSocket();
        ~UDPSocket();

    protected:
        void initSocket();
    };

} // namespace EZSocket

#endif /* EZSOCKET_HXX */