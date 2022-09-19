// File: oculusMessages.hxx

/******************************************************************************
 * Part of this file is taken from Blueprint Subsea's Oculus Viewer software
 * under the following license:
 *
 * *****
 * (c) Copyright 2017 Blueprint Subsea.
 * This file is part of Oculus Viewer
 *
 * Oculus Viewer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Oculus Viewer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * *****
 *
 *****************************************************************************/

#ifndef OCULUSMESSAGES_HXX
#define OCULUSMESSAGES_HXX

#include <cstdint>

#pragma pack(push, 1)

#define OCULUS_CHECK_ID 0x4f53

namespace OculusMessages
{

    enum OculusDeviceType : uint16_t
    {
        deviceTypeUndefined = 0,
        deviceTypeImagingSonar = 1,
    };

    enum OculusMessageType : uint16_t
    {
        messageSimpleFire = 0x15,
        messagePingResult = 0x22,
        messageSimplePingResult = 0x23,
        messageUserConfig = 0x55,
        messageDummy = 0xff,
    };

    enum PingRateType : uint8_t
    {
        pingRateNormal = 0x00,  // 10Hz max ping rate
        pingRateHigh = 0x01,    // 15Hz max ping rate
        pingRateHighest = 0x02, // 40Hz max ping rate
        pingRateLow = 0x03,     // 5Hz max ping rate
        pingRateLowest = 0x04,  // 2Hz max ping rate
        pingRateStandby = 0x05, // Disable ping
    };

    enum DataSizeType : uint8_t
    {
        dataSize8Bit,
        dataSize16Bit,
        dataSize24Bit,
        dataSize32Bit,
    };

    enum OculusPartNumberType : uint16_t
    {
        partNumberUndefined = 0,

        partNumberM370s = 1041,
        partNumberM370s_Artemis = 1229,
        partNumberM370s_Deep = 1217,
        partNumberM373s = 1209,
        partNumberM373s_Deep = 1218,

        partNumberM750d = 1032,
        partNumberM750d_Fusion = 1134,
        partNumberM750d_Artemis = 1135,

        partNumberM1200d = 1042,
        partNumberM1200d_Deep = 1219,
        partNumberM1200d_Artemis = 1228,

        partNumberN1200s = 1220,
        partNumberN1200s_Deep = 1221,

        partNumberEnd = 0xFFFF
    };

    struct OculusMessageHeader
    {
    public:
        uint16_t oculusId;    // Fixed to OCULUS_CHECK_ID
        uint16_t srcDeviceId; // The device id of the source
        uint16_t dstDeviceId; // The device id of the destination
        uint16_t msgId;       // Message identifier
        uint16_t msgVersion;
        uint32_t payloadSize; // The size of the message payload (header not included)
        uint16_t spare2;      // This should be the device part number 
    };

    typedef struct
    {
    public:
        OculusMessageHeader head; // The standard message header

        uint8_t masterMode;      // mode 0 is flexi mode, needs full fire message (not available for third party developers)
                                 // mode 1 - Low Frequency Mode (wide aperture, navigation)
                                 // mode 2 - High Frequency Mode (narrow aperture, target identification)
        PingRateType pingRate;   // Sets the maximum ping rate.
        uint8_t networkSpeed;    // Used to reduce the network comms speed (useful for high latency shared links)
        uint8_t gammaCorrection; // 0 and 0xff = gamma correction = 1.0
                                 // Set to 127 for gamma correction = 0.5
        uint8_t flags;           // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
                                 // bit 1: 0 = 8 bit data, 1 = 16 bit data
                                 // bit 2: 0 = wont send gain, 1 = send gain
                                 // bit 3: 0 = send full return message, 1 = send simple return message
        double range;            // The range demand in percent or m depending on flags
        double gainPercent;      // The gain demand
        double speedOfSound;     // ms-1, if set to zero then internal calc will apply using salinity
        double salinity;         // ppt, set to zero if we are in fresh water
    } OculusSimpleFireMessage;

    typedef struct
    {
        OculusMessageHeader head;
        uint8_t masterMode;
        PingRateType pingRate;
        uint8_t networkSpeed;    /* The max network speed in Mbs , set to 0x00 or 0xff to use link speed */
        uint8_t gammaCorrection; /* The gamma correction - 255 is equal to a gamma correction of 1.0 */
        uint8_t flags;
        double rangePercent; /* The range demand (%) */
        double gainPercent;  /* The percentage gain */
        double speedOfSound; /* The speed of sound - set to zero to use internal calculations */
        double salinity;     /* THe salinity to be used with internal speed of sound calculations (ppt) */
        uint32_t extFlags;
        uint32_t reserved[8];
    } OculusSimpleFireMessage2;

    typedef struct
    {
    public:
        OculusSimpleFireMessage fireMessage;
        uint32_t pingId; /* An incrementing number */
        uint32_t status;
        double frequency;         /* The acoustic frequency (Hz) */
        double temperature;       /* The external temperature (deg C) */
        double pressure;          /* The external pressure (bar) */
        double speeedOfSoundUsed; /* The actual used speed of sound (m/s). May be different to the speed of sound set in the fire message */
        uint32_t pingStartTime;
        DataSizeType dataSize;  /* The size of the individual data entries */
        double rangeResolution; /* The range in metres corresponding to a single range line */
        uint16_t nRanges;       /* The number of range lines in the image*/
        uint16_t nBeams;        /* The number of bearings in the image */
        uint32_t imageOffset;   /* The offset in bytes of the image data from the start of the network message */
        uint32_t imageSize;     /* The size in bytes of the image data */
        uint32_t messageSize;   /* The total size in bytes of the network message */
        // *** NOT ADDITIONAL VARIABLES BEYOND THIS POINT ***
        // There will be an array of bearings (shorts) found at the end of the message structure
        // Allocated at run time
        // short bearings[];
        // The bearings to each of the beams in 0.01 degree resolution
    } OculusSimplePingResult;

    typedef struct
    {
        OculusSimpleFireMessage2 fireMessage;
        uint32_t pingId; /* An incrementing number */
        uint32_t status;
        double frequency;         /* The acoustic frequency (Hz) */
        double temperature;       /* The external temperature (deg C) */
        double pressure;          /* The external pressure (bar) */
        double heading;           /* The heading (degrees) */
        double pitch;             /* The pitch (degrees) */
        double roll;              /* The roll (degrees) */
        double speeedOfSoundUsed; /* The actual used speed of sound (m/s) */
        double pingStartTime;     /* In seconds from sonar powerup (to microsecond resolution) */
        DataSizeType dataSize;    /* The size of the individual data entries */
        double rangeResolution;   /* The range in metres corresponding to a single range line */
        uint16_t nRanges;         /* The number of range lines in the image*/
        uint16_t nBeams;          /* The number of bearings in the image */
        uint32_t spare0;
        uint32_t spare1;
        uint32_t spare2;
        uint32_t spare3;
        uint32_t imageOffset; /* The offset in bytes of the image data from the start */
        uint32_t imageSize;   /* The size in bytes of the image data */
        uint32_t messageSize; /* The total size in bytes of the network message */
        //uint16_t bearings[OSS_MAX_BEAMS]; /* The brgs of the formed beams in 0.01 degree resolution */
    } OculusSimplePingResult2;

    struct OculusVersionInfo
    {
    public:
        uint32_t firmwareVersion0; /* The arm0 firmware version major(8 bits), minor(8 bits), build (16 bits) */
        uint32_t firmwareDate0;    /* The arm0 firmware date */
        uint32_t firmwareVersion1; /* The arm1 firmware version major(8 bits), minor(8 bits), build (16 bits) */
        uint32_t firmwareDate1;    /* The arm1 firmware date */
        uint32_t firmwareVersion2; /* The bitfile version */
        uint32_t firmwareDate2;    /* The bitfile date */
    };

    struct OculusStatusMsg
    {
    public:
        OculusMessageHeader hdr;

        uint32_t deviceId;
        OculusDeviceType deviceType;
        OculusPartNumberType partNumber;
        uint32_t status;
        OculusVersionInfo versinInfo;
        uint32_t ipAddr;
        uint32_t ipMask;
        uint32_t connectedIpAddr;
        uint8_t macAddr0;
        uint8_t macAddr1;
        uint8_t macAddr2;
        uint8_t macAddr3;
        uint8_t macAddr4;
        uint8_t macAddr5;
        double temperature0;
        double temperature1;
        double temperature2;
        double temperature3;
        double temperature4;
        double temperature5;
        double temperature6;
        double temperature7;
        double pressure;
    };

} // namespace OculusMessages

#pragma pack(pop)

#endif
