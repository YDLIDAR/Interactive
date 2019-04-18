/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ETLidarDriver.cpp                                               *
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
#include "SimpleSocket.h"
#include "ETLidarDriver.h"
#include "PassiveSocket.h"
#include "Console.h"
#include "DeviceException.h"
#include <stdio.h>
#include "timer.h"

/*Socket Specific headers */
#include <errno.h>

using namespace ydlidar;


/////////////////////////////////////////////////////////////////////////////////////////
// port defaults to 9000 if not provided.
ETLidarDriver::ETLidarDriver() :
	m_isScanning(false),
	m_isConnected(false),
	offset_len(0),
	m_deviceIp("192.168.0.11"),
	m_port(9000),
	m_sampleRate(20000),
	m_AbnormalCheckCount(2),
	m_force_update(false)
{

	socket_cmd = new CActiveSocket(CSimpleSocket::SocketTypeTcp);
	socket_data = new CPassiveSocket(CSimpleSocket::SocketTypeUdp);
	socket_data->SetSocketType(CSimpleSocket::SocketTypeUdp);
	socket_cmd->SetConnectTimeout(DEFAULT_CONNECT_TIMEOUT_SEC,
								  DEFAULT_CONNECT_TIMEOUT_USEC);
	global_scan_data.data.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////
ETLidarDriver::~ETLidarDriver()
{
	disconnect();
	ScopedLocker data_lock(_data_lock);

	if (socket_data)
	{
		delete socket_data;
		socket_data = NULL;
	}

	ScopedLocker lock(_cmd_lock);

	if (socket_cmd)
	{
		delete socket_cmd;
		socket_cmd = NULL;
	}
}

void ETLidarDriver::WSACleanUp()
{
	CSimpleSocket::WSACleanUp();
}

void ETLidarDriver::updateScanCfg(const lidarConfig &config)
{
	if (m_isConnected)
	{
		return;
	}

	m_force_update = true;
	m_user_config = config;

}

result_t ETLidarDriver::connect(const std::string &ip_address, uint32_t port)
{
	m_deviceIp = ip_address;
	m_isConnected = false;

	if (!configPortConnect(m_deviceIp.c_str(), m_port))
	{
		ydlidar::console.error("%s", DescribeError());
		return RESULT_FAIL;
	}

	lidarConfig config;
	getScanCfg(config);

	if (config.dataRecvPort != port)
	{
		if (!m_force_update)
		{
			m_user_config = config;
			m_user_config.dataRecvPort = port;
			m_force_update = true;
		}
	}

	if (m_force_update)
	{
		setScanCfg(m_user_config);
	}
	else
	{
		m_user_config = config;
	}

	if (!dataPortConnect(m_config.deviceIp, m_config.dataRecvPort))
	{
		stopMeasure();
		ydlidar::console.error("%s", DescribeError(false));
		return RESULT_FAIL;
	}

	m_isConnected = true;
	return RESULT_OK;
}

bool ETLidarDriver::isConnected() const
{
	return m_isConnected;
}

const char *ETLidarDriver::DescribeError(bool isTcp)
{

	if (isTcp)
	{
		ScopedLocker lock(_cmd_lock);
		return socket_cmd != NULL ? socket_cmd->DescribeError() : "NO Socket";
	}
	else
	{
		ScopedLocker lock(_data_lock);
		return socket_data != NULL ? socket_data->DescribeError() : "NO Socket";
	}
}

bool ETLidarDriver::configPortConnect(const char *lidarIP, int tcpPort)
{

	ScopedLocker lock(_cmd_lock);

	if (!socket_cmd)
	{
		return false;
	}

	if (!socket_cmd->IsSocketValid())
	{
		if (!socket_cmd->Initialize())
		{
			return false;
		}
	}
	else
	{
		return socket_cmd->IsSocketValid();
	}

	socket_cmd->SetNonblocking();

	if (!socket_cmd->Open(lidarIP, tcpPort))
	{
		socket_cmd->Close();
		return false;
	}

	socket_cmd->SetReceiveTimeout(DEFAULT_TIMEOUT, 0);
	socket_cmd->SetBlocking();
	return socket_cmd->IsSocketValid();
}

void ETLidarDriver::disConfigConnect()
{
	ScopedLocker lock(_cmd_lock);

	if (!socket_cmd)
	{
		return;
	}

	socket_cmd->Close();
}

void ETLidarDriver::disconnect()
{
	disableDataGrabbing();
	ScopedLocker lock(_data_lock);

	if (!socket_data)
	{
		return;
	}

	socket_data->Close();

}

/*-------------------------------------------------------------
            turnOn
-------------------------------------------------------------*/
bool  ETLidarDriver::turnOn()
{
	if (m_isScanning)
	{
		return true;
	}

	// start scan...
	result_t op_result = startScan();

	if (!IS_OK(op_result))
	{
		op_result = startScan();

		if (!IS_OK(op_result))
		{
			ydlidar::console.error("[CYdLidar] Failed to start scan mode: %x", op_result);
			m_isScanning = false;
			return false;
		}
	}

	if (checkLidarAbnormal())
	{
		stop();
		ydlidar::console.error(
			"[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.");
		m_isScanning = false;
		return false;
	}

	m_isScanning = true;
	ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR is scanning ......");
	fflush(stdout);
	return true;
}

/*-------------------------------------------------------------
            turnOff
-------------------------------------------------------------*/
bool  ETLidarDriver::turnOff()
{
	if (m_isScanning)
	{
		stop();
		ydlidar::console.message("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......");
	}

	m_isScanning = false;
	return true;
}

/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool ETLidarDriver::checkLidarAbnormal()
{

	ydlidar::console.message("Opening scan and checking whether Lidar is abnormal.........");
	int check_abnormal_count = 0;

	if (m_AbnormalCheckCount < 2)
	{
		m_AbnormalCheckCount = 2;
	}

	result_t op_result = RESULT_FAIL;

	while (check_abnormal_count < m_AbnormalCheckCount)
	{
		//Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
		if (check_abnormal_count > 0)
		{
			delay(check_abnormal_count * 1000);
		}

		lidarData scan;
		op_result =  grabScanData(scan);

		if (IS_OK(op_result))
		{
			return false;
		}

		check_abnormal_count++;
	}

	return !IS_OK(op_result);
}

result_t ETLidarDriver::startScan(uint32_t timeout)
{
	result_t ans;

	if (m_isScanning)
	{
		return RESULT_OK;
	}

	stop();
	{
		bool ret = startMeasure();

		if (!ret)
		{
			startMeasure();
		}

		if (!ret)
		{
			return RESULT_FAIL;
		}

		ans = this->createThread();
		return ans;
	}
	return RESULT_OK;
}

bool ETLidarDriver::isScanning() const
{
	return m_isScanning;
}

result_t ETLidarDriver::stop()
{
	disableDataGrabbing();
	bool ret = stopMeasure();

	if (!ret)
	{
		stopMeasure();
	}

	return RESULT_OK;
}

result_t ETLidarDriver::createThread()
{
	_thread = CLASS_THREAD(ETLidarDriver, cacheScanData);

	if (_thread.getHandle() == 0)
	{
		m_isScanning = false;
		return RESULT_FAIL;
	}

	m_isScanning = true;
	return RESULT_OK;
}


char *ETLidarDriver::configMessage(const char *descriptor, char *value)
{
	char buf[100];
	char transDesc[32];
	char recvDesc[32];
	static char recvValue[32];

	strncpy(transDesc, descriptor, sizeof(transDesc));
	valLastName(transDesc);

	sprintf(buf, "%s=%s\n", transDesc, value);
	ScopedLocker lock(_cmd_lock);

	if (!socket_cmd)
	{
		return NULL;
	}

	socket_cmd->Send(reinterpret_cast<uint8_t *>(buf), strlen(buf));

	memset(buf, 0, sizeof(buf));

	if (socket_cmd->Select(0, 50000))
	{
		socket_cmd->Receive(sizeof(buf), reinterpret_cast<uint8_t *>(buf));

		if (2 == sscanf(buf, "%[^=]=%[^=]", recvDesc, recvValue))
		{
			if (!strcmp(transDesc, recvDesc))
			{
				return recvValue;
			}
			else
			{
				return NULL;
			}
		}
		else
		{
			return NULL;
		}

	}
	else
	{
		return value;
	}

	return NULL;
}

bool ETLidarDriver::startMeasure()
{
	bool ret;

	if (!configPortConnect(m_deviceIp.c_str(), m_port))
	{
		ydlidar::console.error("%s", DescribeError());
		return  false;
	}

	lidarConfig cfg;
	ret = configMessage(valName(cfg.motor_en), (char *)configValue[1]) != NULL;
	ret &= configMessage(valName(cfg.laser_en), (char *)configValue[1]) != NULL;
	disConfigConnect();
	return ret;
}

bool ETLidarDriver::stopMeasure()
{
	if (!configPortConnect(m_deviceIp.c_str(), m_port))
	{
		ydlidar::console.error("%s", DescribeError());
		return  false;
	}

	bool ret ;
	lidarConfig cfg;
	ret = configMessage(valName(cfg.motor_en), (char *)configValue[0]) != NULL;
	ret &= configMessage(valName(cfg.laser_en), (char *)configValue[0]) != NULL;
	disConfigConnect();
	return ret;
}

lidarConfig ETLidarDriver::getFinishedScanCfg()
{
	return m_config;
}

bool ETLidarDriver::getScanCfg(lidarConfig &config,
							   const std::string &ip_address)
{
	bool ret = false;

	if (!ip_address.empty())
	{
		m_deviceIp = ip_address;
	}

	lidarConfig cfg;

	if (!configPortConnect(m_deviceIp.c_str(), m_port))
	{
		ydlidar::console.error("%s", DescribeError());
		config = m_config;
		return  ret;
	}

	char *result = configMessage(valName(cfg.laser_en));

	if (result != NULL)
	{
		cfg.laser_en = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.motor_en));

	if (result != NULL)
	{
		cfg.motor_en = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.motor_rpm));

	if (result != NULL)
	{
		cfg.motor_rpm = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.fov_start));

	if (result != NULL)
	{
		cfg.fov_start = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.fov_end));

	if (result != NULL)
	{
		cfg.fov_end = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.trans_sel));

	if (result != NULL)
	{
		cfg.trans_sel = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.dataRecvPort));

	if (result != NULL)
	{
		cfg.dataRecvPort = atoi(result);
		ret = true;
	}


	result = configMessage(valName(cfg.dhcp_en));

	if (result != NULL)
	{
		cfg.dhcp_en = atoi(result);
		ret = true;
	}

	result = configMessage(valName(cfg.dataRecvIp));

	if (result != NULL)
	{
		strcpy(cfg.dataRecvIp, result);
		ret = true;
	}

	result = configMessage(valName(cfg.deviceIp));

	if (result != NULL)
	{
		strcpy(cfg.deviceIp, result);
		ret = true;
	}

	result = configMessage(valName(cfg.deviceNetmask));

	if (result != NULL)
	{
		strcpy(cfg.deviceNetmask, result);
		ret = true;
	}


	result = configMessage(valName(cfg.deviceGatewayIp));

	if (result != NULL)
	{
		strcpy(cfg.deviceGatewayIp, result);
		ret = true;
	}

	result = configMessage(valName(cfg.laserScanFrequency));

	if (result != NULL)
	{
		cfg.laserScanFrequency = atoi(result);
		m_sampleRate = 1000 / cfg.laserScanFrequency * 1000;
		ret = true;
	}

	if (ret)
	{
		m_config = cfg;
		config = cfg;
	}

	disConfigConnect();
	return ret;
}


void ETLidarDriver::setScanCfg(const lidarConfig &config)
{
	char str[32];

	if (!configPortConnect(m_deviceIp.c_str(), m_port))
	{
		ydlidar::console.error("%s", DescribeError());
		return ;
	}

	char *result = NULL;

	if (m_config.motor_rpm != config.motor_rpm)
	{
		_itoa(config.motor_rpm, str, 10);
		result = configMessage(valName(config.motor_rpm), str);

		if (result != NULL)
		{
			m_config.motor_rpm = atoi(result);
			ydlidar::console.message("reset motor RPM[%d] successfully!",
									 m_config.motor_rpm);
		}
	}


	if (m_config.fov_start != config.fov_start)
	{
		_itoa(config.fov_start, str, 10);
		result = configMessage(valName(config.fov_start), str);

		if (result != NULL)
		{
			m_config.fov_start = atoi(result);
		}
	}


	if (m_config.fov_end != config.fov_end)
	{
		_itoa(config.fov_end, str, 10);
		result = configMessage(valName(config.fov_end), str);

		if (result != NULL)
		{
			m_config.fov_end = atoi(result);
		}
	}


	if (m_config.dataRecvPort != config.dataRecvPort)
	{
		_itoa(config.dataRecvPort, str, 10);
		result = configMessage(valName(config.dataRecvPort), str);

		if (result != NULL)
		{
			m_config.dataRecvPort = atoi(result);
			ydlidar::console.message("reset data Port[%d] successfully!",
									 m_config.dataRecvPort);
		}
	}

	if (strcmp(m_config.dataRecvIp, config.dataRecvIp))
	{
		result = configMessage(valName(config.dataRecvIp), (char *)config.dataRecvIp);

		if (result != NULL)
		{
			ydlidar::console.message("reset data IP[%s] successfully!",
									 m_config.dataRecvIp);
			strcpy(m_config.dataRecvIp, result);
		}
	}

	disConfigConnect();
}



bool ETLidarDriver::dataPortConnect(const char *lidarIP, int localPort)
{
	ScopedLocker lock(_data_lock);

	if (!socket_data)
	{
		return false;
	}

	if (!socket_data->IsSocketValid())
	{
		if (socket_data->Initialize())
		{
			if (!socket_data->Listen(NULL, localPort))
			{
				socket_data->Close();
				return false;
			}

			socket_data->SetReceiveTimeout(DEFAULT_TIMEOUT, 0);
		}
	}

	return socket_data->IsSocketValid();
}

void ETLidarDriver::disableDataGrabbing()
{
	{
		ScopedLocker l(_lock);

		if (m_isScanning)
		{
			m_isScanning = false;
			_dataEvent.set();
		}
	}
	_thread.join();
}


result_t ETLidarDriver::grabScanData(lidarData &scan, uint32_t timeout)
{
	switch (_dataEvent.wait(timeout))
	{
	case Event::EVENT_TIMEOUT:
		return RESULT_TIMEOUT;

	case Event::EVENT_OK:
	{
		if (global_scan_data.data.size() == 0)
		{
			return RESULT_FAIL;
		}

		ScopedLocker l(_lock);
		scan = global_scan_data;
		global_scan_data.data.clear();
	}

	return RESULT_OK;

	default:
		return RESULT_FAIL;
	}

}


int ETLidarDriver::cacheScanData()
{

	lidarData scandata;
	int timeout = 0;
	scandata.data.clear();

	while (m_isScanning)
	{
		try
		{
			lidarData local_data;
			int size = getScanData(local_data);

			if (size > 0)
			{
				if (local_data.headFrameFlag)
				{
					if (scandata.data.size())
					{
						if (scandata.headFrameFlag)
						{
							double angle_diff = m_config.fov_end - m_config.fov_start;

							if (angle_diff > 0)
							{
								double echo_angle = angle_diff / scandata.data.size();
								offset_len = (360 - angle_diff) / echo_angle;
							}

							scandata.scan_time += 1e9 / m_sampleRate * offset_len;
							_lock.lock();//timeout lock, wait resource copy
							global_scan_data = scandata;
							_dataEvent.set();
							_lock.unlock();
						}
					}

					scandata.data.clear();
					scandata = local_data;
					scandata.system_timestamp = local_data.system_timestamp - 1e9 / m_sampleRate *
												local_data.data.size();
				}
				else
				{
					scandata.scan_time += local_data.scan_time;
					scandata.data.insert(scandata.data.end(), local_data.data.begin(),
										 local_data.data.end());
				}
			}
			else
			{
				ydlidar::console.error("Failed to get ScanData[%d]", size);
			}

			timeout = 0;
		}
		catch (TimeoutException &e)
		{
			timeout++;
			scandata.data.clear();
			ydlidar::console.error("timeout[%d]:%s", timeout, e.what());

			if (timeout > DEFAULT_TIMEOUT_COUNT)
			{
				m_isScanning = false;
			}
			else
			{
				continue;
			}

		}
		catch (CorruptedDataException &e)
		{
			ydlidar::console.error("scan data parse error: %s", e.what());
			continue;
		}
		catch (DeviceException &e)
		{
			ydlidar::console.error("%s", e.what());
			m_isScanning = false;
		}
		catch (...)
		{
			m_isScanning = false;
		}
	}

	ydlidar::console.message("scanning thread exiting....");
	return 0;
}


int ETLidarDriver::getScanData(lidarData &data)
{

	int offset;


	/* wait data from socket. */
	{
		ScopedLocker lock(_data_lock);

		if (!socket_data)
		{
			return -1;
		}

		if (socket_data->Receive(sizeof(frame.frameBuf),
								 reinterpret_cast<uint8_t *>(frame.frameBuf)) < 0)
		{
			throw TimeoutException(DescribeError(false));
		}
	}


	data.system_timestamp = CStatTimer::GetCurrentTime();
	/* check frame head */
	frame.frameHead = DSL(frame.frameBuf[0], 8) | DSL(frame.frameBuf[1], 0);

	if (FRAME_PREAMBLE != frame.frameHead)
	{
		throw CorruptedDataException("recv data error for header");
	}

	/* check device type */
	frame.deviceType = (frame.frameBuf[2] >> 4) & 0xf;

	if (LIDAR_2D != frame.deviceType)
	{
		throw CorruptedDataException("recv data error for device type");
	}

	/* check frame type */
	frame.frameType = frame.frameBuf[2] & 0xf;

	if (DATA_FRAME != frame.frameType)
	{
		throw CorruptedDataException("recv data error for frame type");
	}

	/* parser head length */
	frame.dataIndex = (frame.frameBuf[3] >> 4) & 0xf;
	frame.dataIndex = (frame.dataIndex + 1) * 4;

	/* parser frame index */
	frame.frameIndex = frame.frameBuf[3] & 0xf;

	/* parser timestamp */
	frame.timestamp = DSL(frame.frameBuf[4], 24) | DSL(frame.frameBuf[5], 16)
					  | DSL(frame.frameBuf[6], 8)  | DSL(frame.frameBuf[7], 0);

	/* parser head frame flag */
	frame.headFrameFlag = (frame.frameBuf[8] >> 4) & 0xf;

	/* parser data format */
	frame.dataFormat = frame.frameBuf[8] & 0xf;

	/* parser distance scale */
	frame.disScale = frame.frameBuf[9];

	/* parser start angle */
	frame.startAngle = DSL(frame.frameBuf[10], 8) | DSL(frame.frameBuf[11], 0);

	/* parser valid data number */
	frame.dataNum = DSL(frame.frameBuf[12], 8) | DSL(frame.frameBuf[13], 0);

	/* parser frame crc */
	frame.frameCrc = DSL(frame.frameBuf[14], 8) | DSL(frame.frameBuf[15], 0);

	/* parser data */
	data.data.resize(frame.dataNum);
	lidarPot pot;

	if (frame.dataFormat == 0)
	{
		for (uint32_t i = 0; i < frame.dataNum; i++)
		{
			offset = frame.dataIndex + 4 * i;
			pot.intensity = (uint16_t)(DSL(frame.frameBuf[offset],
										   8) | DSL(frame.frameBuf[offset + 1], 0));
			pot.range = (float)(DSL(frame.frameBuf[offset + 2],
									8) | DSL(frame.frameBuf[offset + 3], 0)) / 1000.f;

			if (i > 0)
			{
				pot.angle = (float)(frame.frameCrc - frame.startAngle) /
							(frame.dataNum - 1) / 100.f + data.data[i - 1].angle;

				if (pot.angle >= 360)
				{
					pot.angle -= 360;
				}
			}
			else
			{
				pot.angle = (float)frame.startAngle / 100.f;
			}

			data.data[i] = pot;

		}

	}
	else
	{
		for (unsigned int i = 0; i < frame.dataNum; i++)
		{
			offset = frame.dataIndex + 4 * i;
			pot.intensity = (uint16_t)frame.frameBuf[offset];
			pot.range = (float)(DSL(frame.frameBuf[offset + 2],
									8) | DSL(frame.frameBuf[offset + 3], 0)) / 1000.f;

			if (i > 0)
			{
				pot.angle = (float)(frame.frameBuf[offset + 1]) / 100.f + data.data[i -
																					1].angle;

				if (pot.angle >= 360)
				{
					pot.angle -= 360;
				}
			}
			else
			{
				pot.angle = (float)frame.startAngle / 100.f;
			}

			data.data[i] = pot;
		}
	}

	data.self_timestamp = (uint64_t)(frame.timestamp * 100);
	data.headFrameFlag = (int)frame.headFrameFlag;
	data.scan_time     = 1e9 / m_sampleRate * frame.dataNum;

	return frame.dataNum;

}
