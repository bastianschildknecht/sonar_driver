// File: oculusMessages.hxx

#ifndef OCULUSMESSAGES_HXX
#define OCULUSMESSAGES_HXX

#include <cstdint>

#pragma pack(push, 1)

#define OCULUS_CHECK_ID 0x4f53

namespace OculusMessages
{
    struct OculusMessageHeader
    {
    public:
        uint16_t oculusId;    // Fixed to OCULUS_CHECK_ID
        uint16_t srcDeviceId; // The device id of the source
        uint16_t dstDeviceId; // The device id of the destination
        uint16_t msgId;       // Message identifier
        uint16_t msgVersion;
        uint32_t payloadSize; // The size of the message payload (header not included)
        uint16_t spare2;
    };

} // namespace OculusMessages

#pragma pack(pop)

#endif