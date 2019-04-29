/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ETLidarDriver.h                                                 *
*  @brief    TOF LIDAR DRIVER                                                *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuifurong618@eaibot.com                                     *
*  @version  1.0.0(版本号)                                                    *
*  @date     chushuifurong618@eaibot.com                                     *
*                                                                            *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2018/08/09 | 1.0.0     | Tony.Yang      | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#pragma once

/* Header file to enable threading and ergo callback */
#include "etlidar_protocol.h"
#include "utils.h"
#include "thread.h"
#include "locker.h"
#include <vector>
/* Header files for socket variable */


namespace ydlidar
{

class CActiveSocket;
class CPassiveSocket;


class YDLIDAR_API ETLidarDriver
{
public:

	/**
	 * @brief ETLidarDriver
	 * @param lidarIP
	 * @param port
	 */
	explicit ETLidarDriver();

	~ETLidarDriver();


	/**
	* @brief WSACleanUp for windows
	*/
	static void WSACleanUp();

	/**
	 * @brief connect
	 * @param ip_address
	 * @param port
	 * @return
	 */
	result_t connect(const std::string &ip_address, uint32_t port = 8000);

	/**
	 * @brief isconnected
	 * @return
	 */
	bool isConnected() const;
	/**
	* @brief Disconnect from ETLidar device.
	*/
	void disconnect();

	/**
	* @brief Get current scan configuration.
	* @returns scanCfg structure.
	*/
	int getScanCfg(lidarConfig &config, const std::string &ip_address = "");

	/**
	* @brief Get current scan update configuration.
	* @returns scanCfg structure.
	*/
	lidarConfig getFinishedScanCfg();

	/**
	 * @brief updateScanCfg
	 * @param config
	 */
	void updateScanCfg(const lidarConfig &config);

	/**
	* @brief Set scan configuration.
	* @param cfg structure containing scan configuration.
	*/
	void setScanCfg(const lidarConfig &config);


	/** returns true if the lidar data is normal, If it's not*/
	bool checkLidarAbnormal();

	/** Turn on the motor and lidar enable*/
	bool  turnOn();  //!< See base class docs

	/** Turn off the motor enable and close the scan */
	bool  turnOff(); //!< See base class docs

	/**
	 * @brief isscanning
	 * @return
	 */
	bool isScanning() const;

	/**
	 * @brief stop
	 * @return
	 */
	result_t stop();

	/**
	 * @brief grabScanData
	 * @param scan
	 * @param timeout
	 * @return
	 */
	result_t grabScanData(lidarData &scan, uint32_t timeout = DEFAULT_TIMEOUT) ;

	/**
	 * @brief get socket describe error
	 * @param isTcp tcp/udp socket
	 * @return
	 */
	const char *DescribeError(bool isTcp = true);


private:

	/**
	 * @brief startScan
	 * @param timeout
	 * @return
	 */
	result_t startScan(uint32_t timeout = DEFAULT_TIMEOUT) ;
	/**
	* @brief Connect config port to ETLidar.
	* @param remote IP & port.
	*/
	bool configPortConnect(const char *lidarIP, int tcpPort = 9000);

	void disConfigConnect();

	/**
	* @brief Disconnect from ETLidar device.
	*/
	char *configMessage(const char *descriptor, char *value = NULL);

	/**
	* @brief Start measurements.
	* After receiving this command ETLidar unit starts spinning laser and measuring.
	*/
	bool startMeasure();

	/**
	* @brief Stop measurements.
	* After receiving this command ETLidar unit stop spinning laser and measuring.
	*/
	bool stopMeasure();

	/**
	* @brief Connect data port to ETLidar.
	* @param remote IP & local port.
	*/
	bool dataPortConnect(const char *lidarIP, int localPort = 8000);

	/**
	 * @brief createThread
	 * @return
	 */
	result_t createThread();

	/**
	 * @brief disableDataGrabbing
	 */
	void disableDataGrabbing();
	/**
	* @brief Receive scan message.
	*
	* @param data pointer to lidarData buffer structure.
	*/
	int getScanData(lidarData &data);

	/**
	* @brief parsing scan \n
	*/
	int cacheScanData();
private:
	/* Variable for LIDAR compatibility */
	bool            m_isScanning;
	bool            m_isConnected;
	Event           _dataEvent;      ///<
	Locker          _lock;        ///<
	Thread          _thread;        ///<
	Locker          _cmd_lock;        ///<
	Locker          _data_lock;       ///<


	lidarData       global_scan_data;
	lidarConfig     m_config;
	lidarConfig     m_user_config;
	size_t          offset_len;
	int             m_AbnormalCheckCount;
	bool            m_force_update;

	enum
	{
		DEFAULT_TIMEOUT   = 2000,    /**< 默认超时时间. */
		DEFAULT_TIMEOUT_COUNT = 10,
	};


	/* ETLidar specific Variables */
	std::string               m_deviceIp;
	int                       m_port;
	int                       m_sampleRate;
	/* Sockets for ydlidar */
	CActiveSocket             *socket_cmd;
	CPassiveSocket            *socket_data;
	dataFrame                 frame;
	const char               *configValue[2] = {"0", "1"};

};

} /* namespace */
