// File: ezsocket.hxx

#ifndef EZSOCKET_HXX
#define EZSOCKET_HXX

#include <cstdint>
#include <mutex>

#ifdef _WIN32

#include <winsock2.h>

#else

#include <netinet/in.h>

#endif

#define STR_ADDRESS_LEN 32

namespace EZSocket
{

    enum SocketState
    {
        Exists,
        Ready,
        Connected,
        Bound,
        InitError,
        ConnectError,
        BindError,
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
        virtual void connectToHost(const std::string& address, uint16_t port);

        // Disconnects the socket from the host
        // (Only needed if you want to reconnect or change receive buffer size)
        virtual void disconnect();

        // Binds the socket to the specified address (can be "ANY") for listening
        virtual void bindToAddress(const std::string& address, uint16_t port);

        // Reads available bytes from the connected socket into a buffer up to a maximum length
        virtual int32_t readData(void *buffer, int32_t maxLength);

        // Waits for a message and reads available bytes from the bound socket into a buffer up to a maximum length
        // Also retreives the sender address and stores it in addressBuffer if not NULL
        virtual int32_t waitForDataAndAddress(void *buffer, int32_t maxLength, std::string& addressBuffer, size_t bufferLength);

        // Writes a specified amount of bytes to the connected socket
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
        std::mutex *mutex;
        struct sockaddr_in host_addr;
        struct sockaddr_in server_addr;
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