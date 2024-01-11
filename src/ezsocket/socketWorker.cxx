// File: socketWorker.cxx

#include <stdint.h>
#include <sonar_driver/ezsocket/ezsocket.hxx>
#include <sonar_driver/ezsocket/socketWorker.hxx>

using namespace EZSocket;

SocketWorker::SocketWorker(Socket *socket, uint32_t readBufferSize, uint32_t writeBufferSize)
{
    SocketWorker::socket = socket;
    SocketWorker::readBufferSize = readBufferSize;
    SocketWorker::writeBufferSize = writeBufferSize;
    readBuffer = new uint8_t[readBufferSize];
    writeBuffer = new uint8_t[writeBufferSize];
    readPos = 0;
    sockReadPos = 0;
    writePos = 0;
    sockWritePos = 0;
    bytesToRead = 0;
    bytesToWrite = 0;
    readMutex = new std::mutex();
    writeMutex = new std::mutex();
    workerRunning = false;
}

SocketWorker::~SocketWorker()
{
    if (workerRunning)
    {
        delete workerThread;
        workerThread = nullptr;
    }
    delete[] readBuffer;
    readBuffer = nullptr;
    delete[] writeBuffer;
    writeBuffer = nullptr;
    delete readMutex;
    readMutex = nullptr;
    delete writeMutex;
    writeMutex = nullptr;
}

bool SocketWorker::startWorker()
{
    SocketState state = socket->getState();
    if (!workerRunning && (state == SocketState::Connected || state == SocketState::Bound))
    {
        workerThread = new std::thread([this] { this->startWorker(); });
        workerRunning = true;
        return true;
    }
    else
    {
        return false;
    }
}

bool SocketWorker::queueForWrite(const void *buffer, uint32_t length)
{
    writeMutex->lock();
    if (getFreeWriteSpace() >= length)
    {
        copyToRingBuffer(writeBuffer, buffer, length, writeBufferSize, &writePos, &sockWritePos);
        bytesToWrite += length;
        writeMutex->unlock();
        return true;
    }
    else
    {
        writeMutex->unlock();
        return false;
    }
}

uint32_t SocketWorker::readFromQueue(void *buffer, uint32_t maxLenght)
{
    readMutex->lock();
    if (bytesToRead > 0)
    {
        uint32_t copyBytes = bytesToRead < maxLenght ? bytesToRead : maxLenght;
        copyFromRingBuffer(readBuffer, buffer, copyBytes, readBufferSize, &sockReadPos, &readPos);
        bytesToRead -= copyBytes;
        readMutex->unlock();
        return copyBytes;
    }
    else
    {
        readMutex->unlock();
        return 0;
    }
}

uint32_t SocketWorker::bytesAvailable()
{
    return bytesToRead;
}

void SocketWorker::worker()
{
    uint32_t recvBuffSize = 2048;
    void *recvBuff = malloc(recvBuffSize);
    uint32_t sendBuffSize = 2048;
    void *sendBuff = malloc(sendBuffSize);
    uint64_t recvBytes;
    uint32_t readyBytes;
    int32_t recvSize;
    int32_t sendSize;
    while (true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        /**
         *  Receive availble bytes from socket and store them in the readBuffer
         **/
        recvBytes = socket->bytesAvailable();
        if (recvBytes > 0 && recvBytes <= getFreeReadSpace())
        {
            // Check if receive buffer is still big enough and expand if needed
            if (recvBytes > recvBuffSize)
            {
                recvBuff = realloc(recvBuff, recvBytes);
                recvBuffSize = recvBytes;
            }

            // Read data from network
            recvSize = socket->readData(recvBuff, recvBytes);

            // Check for socket errors
            if (recvSize >= 0)
            {
                readMutex->lock();
                copyToRingBuffer(readBuffer, recvBuff, recvSize, readBufferSize, &sockReadPos, &readPos);
                bytesToRead += recvSize;
                readMutex->unlock();
            }
            else
            {
                // Socket no longer connected, exit thread
                break;
            }
        }

        /**
         *  Send bytes to write over the socket
         **/
        // Only read available bytes to write once to ensure consistency
        readyBytes = bytesToWrite;
        if (readyBytes > 0)
        {
            // Check if write buffer is still big enough and expand if needed
            if (readyBytes > sendBuffSize)
            {
                sendBuff = realloc(sendBuff, readyBytes);
                sendBuffSize = readyBytes;
            }

            // Copy data to send from ring buffer
            writeMutex->lock();
            copyFromRingBuffer(writeBuffer, sendBuff, readyBytes, writeBufferSize, &writePos, &sockWritePos);
            bytesToWrite -= readyBytes;
            writeMutex->unlock();

            // Send data over the socket
            sendSize = socket->writeData(sendBuff, readyBytes);

            // Check for socket errors
            if (sendSize != static_cast<int32_t>(readyBytes))
            {
                // Socket problem, stop thread
                break;
            }
        }
    }
    free(recvBuff);
    recvBuff = nullptr;
    free(sendBuff);
    sendBuff = nullptr;
}

uint32_t SocketWorker::getFreeReadSpace()
{
    return readBufferSize - bytesToRead;
}

uint32_t SocketWorker::getFreeWriteSpace()
{
    return writeBufferSize - bytesToWrite;
}

void SocketWorker::copyToRingBuffer(uint8_t *buffer, const void *src, uint32_t length, uint32_t bufferSize, uint32_t *writerPos, const uint32_t *readerPos)
{
    // WARNING: This method assumes that there is enough space available in the buffer! Check before calling!

    // Check if writer is in front of reader (written bytes behind writer)
    if (*writerPos >= *readerPos)
    {
        // Calculate remaining space until end of buffer
        uint32_t rest = bufferSize - *writerPos;
        // Check if we can write everything in one go
        if (length < rest)
        {
            // Data fits into the rest of the buffer
            memcpy(&buffer[*writerPos], src, length);
            *writerPos += length;
        }
        else
        {
            // Data is to big for the rest of the buffer, therefore split it in two
            memcpy(&buffer[*writerPos], src, rest);
            uint32_t split2 = length - rest;
            memcpy(buffer, src, split2);
            *writerPos = split2;
        }
    }
    else
    {
        // Writer is behind reader, therefore we can copy directly
        memcpy(&buffer[*writerPos], src, length);
        *writerPos += length;
    }
}

void SocketWorker::copyFromRingBuffer(const uint8_t *buffer, void *dst, uint32_t length, uint32_t bufferSize, const uint32_t *writerPos, uint32_t *readerPos)
{
    // WARNING: This method assumes that there are bytes to read and that the length is the minimum of available bytes and the requested bytes!
    // Ceck before calling!

    // Check if reader is behind writer
    if (*readerPos < *writerPos)
    {
        // We can copy directly because the writer is ahead
        memcpy(dst, &buffer[*readerPos], length);
        *readerPos += length;
    }
    else
    {
        // reader is ahead of writer in terms of buffer index
        // Calculate remaining bytes until end of buffer
        uint32_t rest = bufferSize - *readerPos;
        // Check if we need to read less than that
        if (length <= rest)
        {
            // We can copy in one go
            memcpy(dst, &buffer[*readerPos], length);
            *readerPos += length;
        }
        else
        {
            // We need to read more than until to the buffer end, therefore split into two reads
            memcpy(dst, &buffer[*readerPos], rest);
            uint32_t split2 = length - rest;
            uint8_t *dst_bytes = (uint8_t *)dst;
            memcpy(&dst_bytes[rest], buffer, split2);
            *readerPos = split2;
        }
    }
}