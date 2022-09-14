
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"

XdaInterface::XdaInterface(rclcpp::Node::SharedPtr node_ptr)
  : node(node_ptr),
    m_device(nullptr)
{
	RCLCPP_INFO(logger(), "Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);

	node->declare_parameter<int>("publisher_queue_size", 5);
	node->declare_parameter<std::string>("frame_id", DEFAULT_FRAME_ID);
	node->declare_parameter<double>("expected_frequency", 100.0);

	node->declare_parameter<int>("baudrate", 0);
	node->declare_parameter<std::string>("device_id", "");
	node->declare_parameter<std::string>("port", "");
	node->declare_parameter<std::string>("log_file", "");

	registerPublishers();
}

XdaInterface::~XdaInterface()
{
  RCLCPP_INFO(logger(), "Cleaning up ...");
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)
		{
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

void XdaInterface::registerPublishers()
{
	registerImpl<ImuPublisher>("pub_imu");

	registerImpl<OrientationPublisher>("pub_quaternion");

	registerImpl<AccelerationPublisher>("pub_acceleration");

	registerImpl<AngularVelocityPublisher>("pub_angular_velocity");

	registerImpl<MagneticFieldPublisher>("pub_mag");

	registerImpl<OrientationIncrementsPublisher>("pub_dq");

	registerImpl<VelocityIncrementPublisher>("pub_dv");

	registerImpl<TimeReferencePublisher>("pub_sampletime");

	registerImpl<TemperaturePublisher>("pub_temperature");

	registerImpl<PressurePublisher>("pub_pressure");

	registerImpl<GnssPublisher>("pub_gnss");

	registerImpl<TwistPublisher>("pub_twist");

	registerImpl<FreeAccelerationPublisher>("pub_free_acceleration");

	registerImpl<TransformPublisher>("pub_transform");

	registerImpl<PositionLLAPublisher>("pub_positionLLA");

	registerImpl<VelocityPublisher>("pub_velocity");
}

bool XdaInterface::connectDevice()
{
	// Read baudrate parameter if set
	XsBaudRate baudrate = XBR_Invalid;
	int baudrateParam = 0;
	node->get_parameter("baudrate", baudrateParam);
	if (baudrateParam != 0)
	{
		RCLCPP_INFO(logger(), "Found baudrate parameter: %d", baudrateParam);
		baudrate = XsBaud::numericToRate(baudrateParam);
	}

	// Read device ID parameter
	bool checkDeviceID = false;
	std::string deviceId;
	node->get_parameter("device_id", deviceId);
	if (!deviceId.empty())
	{
		checkDeviceID = true;
		RCLCPP_INFO(logger(), "Found device ID parameter: %s.",deviceId.c_str());
	}

	// Read port parameter if set
	XsPortInfo mtPort;
	std::string portName;
	node->get_parameter("port", portName);
	if (!portName.empty())
	{
		RCLCPP_INFO(logger(), "Found port name parameter: %s", portName.c_str());
		mtPort = XsPortInfo(portName, baudrate);
		RCLCPP_INFO(logger(), "Scanning port %s ...", portName.c_str());
		if (!XsScanner::scanPort(mtPort, baudrate))
			return handleError("No MTi device found. Verify port and baudrate.");
		if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
			return handleError("No MTi device found with matching device ID.");
	}
	else
	{
		RCLCPP_INFO(logger(), "Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found.");

	RCLCPP_INFO(logger(), "Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	RCLCPP_INFO(logger(), "Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	RCLCPP_INFO(logger(), "Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	RCLCPP_INFO(logger(), "Measuring ...");
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	std::string logFile;
	node->get_parameter("log_file", logFile);
	if (!logFile.empty())
	{
		if (m_device->createLogFile(logFile) != XRV_OK)
			return handleError("Failed to create a log file! (" + logFile + ")");
		else
	RCLCPP_INFO(logger(), "Created a log file: %s", logFile.c_str());

	RCLCPP_INFO(logger(), "Recording to %s ...", logFile.c_str());
		if (!m_device->startRecording())
			return handleError("Could not start recording");
	}

	return true;
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
	RCLCPP_ERROR(logger(), "%s", error.c_str());
	close();
	return false;
}
