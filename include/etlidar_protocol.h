#pragma once
#include "v8stdint.h"
#include <vector>

#if defined(_WIN32)
#if defined(ETLidar_STATIC)
#define ETLidar_EXPORT
#elif defined(ETLidar_EXPORTS)
#define ETLidar_EXPORT __declspec(dllexport)
#else
#define ETLidar_EXPORT __declspec(dllimport)
#endif

#else

#include <stdint.h>
#define ETLidar_EXPORT
#define _itoa(value, str, radix) {sprintf(str, "%d", value);}

#endif // ifdef WIN32


#define valName(val) (#val)
#define valLastName(val) \
  { \
    char* strToken; \
    char str[64]; \
    strncpy(str, (const char*)val, sizeof(str)); \
    strToken = strtok(str, "."); \
    while (strToken != NULL) { \
      strcpy(val, (const char*)strToken); \
      strToken = strtok(NULL, "."); \
    } \
  }


/**
 * @class dataFrame
 * @brief data frame Structure.
 *
 * @author jzhang
 */
#define FRAME_PREAMBLE 0xFFEE
#define LIDAR_2D 0x2
#define DATA_FRAME 0x1
#define DEFAULT_INTENSITY 10
#define DSL(c, i) ((c << i) & (0xFF << i))

#define DEFAULT_CONNECT_TIMEOUT_SEC 2
#define DEFAULT_CONNECT_TIMEOUT_USEC 0

#define SDK_VERSION "1.1"


namespace ydlidar
{


typedef struct _dataFrame
{
	uint16_t frameHead;
	uint8_t deviceType;
	uint8_t frameType;
	uint8_t dataIndex;
	uint8_t frameIndex;
	uint32_t timestamp;
	uint8_t headFrameFlag;
	uint8_t dataFormat;
	uint8_t disScale;
	uint32_t startAngle;
	uint32_t dataNum;
	uint32_t frameCrc;
	char frameBuf[2048];
} dataFrame;

/**
 * @class lidarConfig
 * @brief Structure containing scan configuration.
 *
 * @author jzhang
 */
typedef struct _lidarConfig
{
	/**
	 * @brief Scanning enable.
	 */
	int laser_en;

	/**
	 * @brief rotate enable.
	 */
	int motor_en;

	/**
	 * @brief motor RPM.
	 */
	int motor_rpm;

	/**
	 * @brief start FOV angle.
	 */
	int fov_start;

	/**
	 * @brief end FOV angle.
	 */
	int fov_end;

	/**
	 * @brief data receive interface, USB or Ethernet.
	 */
	int trans_sel;

	/**
	 * @brief data receive IP.
	 */
	char dataRecvIp[16];

	/**
	 * @brief data receive PORT.
	 */
	int dataRecvPort;

	/**
	 * @brief device network config, HDCP or Manual.
	 */
	int dhcp_en;

	/**
	 * @brief device IP.
	 */
	char deviceIp[16];

	/**
	 * @brief device netmask.
	 */
	char deviceNetmask[16];

	/**
	 * @brief device gateway ip.
	 */
	char deviceGatewayIp[16];

	int laserScanFrequency;
} lidarConfig;



/**
 * @class lidarPot
 * @brief Structure containing single scan point.
 *
 * @author Tony.Yang
 */
typedef struct _lidarPot
{
	/**
	 * @brief range
	 */
	float range;
	/**
	 * @brief angle
	 */
	float angle;
	/**
	 * @brief intensity
	 */
	int   intensity;
} lidarPot;

/**
 * @class lidarData
 * @brief Structure containing single scan message.
 *
 * @author jzhang
 */
typedef struct _lidarData
{

	/**
	 * @brief ranges.
	 */
	std::vector<lidarPot> data;

	/**
	 * @brief headFrameFlag.
	 */
	int headFrameFlag;

	/**
	 * @brief frame timestamp in nanoseconds.
	 */
	uint64_t self_timestamp;

	/**
	 * @brief system time.
	 */
	uint64_t system_timestamp;

	/**
	 * @brief scan_time
	 */
	uint64_t scan_time;

} lidarData;

}

