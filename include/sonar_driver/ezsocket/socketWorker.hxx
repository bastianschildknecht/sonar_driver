#pragma once

#include <stdint.h>
#include <thread>
#include <memory.h>
#include <chrono>
#include <mutex>

#include <sonar_driver/ezsocket/ezsocket.hxx>


namespace EZSocket
{
    class SocketWorker
    {
    public:
        SocketWorker(Socket *socket, uint32_t readBufferSize, uint32_t writeBufferSize);
        virtual ~SocketWorker();
        virtual bool startWorker();
        virtual bool queueForWrite(const void *buffer, uint32_t length);
        virtual uint32_t readFromQueue(void *buffer, uint32_t maxLenght);
        virtual uint32_t bytesAvailable();

    protected:
        Socket *socket;
        uint32_t readBufferSize;
        uint32_t writeBufferSize;
        uint32_t bytesToRead;
        uint32_t bytesToWrite;
        uint8_t *readBuffer;
        uint8_t *writeBuffer;
        uint32_t readPos;
        uint32_t writePos;
        uint32_t sockReadPos;
        uint32_t sockWritePos;
        bool workerRunning;
        std::thread *workerThread;
        std::mutex *writeMutex;
        std::mutex *readMutex;
        void worker();
        uint32_t getFreeReadSpace();
        uint32_t getFreeWriteSpace();
        static void copyToRingBuffer(uint8_t *buffer, const void *src, uint32_t length, uint32_t bufferSize, uint32_t *writerPos, const uint32_t *readerPos);
        static void copyFromRingBuffer(const uint8_t *buffer, void *dst, uint32_t length, uint32_t bufferSize, const uint32_t *writerPos, uint32_t *readerPos);
    };
} // namespace EZSocket
