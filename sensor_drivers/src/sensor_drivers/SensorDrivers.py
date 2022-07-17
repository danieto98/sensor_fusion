#!/usr/bin/env python3

import re
import rospy
import serial
import time

from sensor_drivers.IMUDriver import IMUDriver
from std_msgs.msg import String

class SensorDrivers:
	def __init__(self):
		self.IMU = IMUDriver()

		self.transmitData = False

		self.port = rospy.get_param('port', '/dev/ttyACM0')
		self.baud_rate = str(rospy.get_param('baud_rate', '230400'))

		# Init. serial connection
		self.arduino = serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=1.0)

		# Sleep for one second
		time.sleep(1)

		# Flush previous partial messages
		self.arduino.readline()

		# Get config.
		if not self.getConfig():
			rospy.signal_shutdown('Error accessing sensors')

		# Start transmission of sensor data
		self.startDataTransmission()

	def getConfig(self):
		msg = ""
		if self.arduino.in_waiting:
			msg = str(self.arduino.readline(), 'utf-8')

		if re.search("^Error:.*\\r\\n$", msg) != None:
			rospy.logerr(msg[6:])
			return False
		else:
			self.arduino.write(bytes(b'GET_CONFIG\r\n'))

			msg = str(self.arduino.readline(), 'utf-8')

			if re.search("^Config:[0-9]+,[0-9]+,[0-9]+\\r\\n$", msg) != None:
				cfg = [int(e) for e in msg[7:].split(',')]
				self.IMU.config(cfg[0], cfg[1], cfg[2])
				return True
			else:
				rospy.logerr("Unrecognized message from sensors. Please reset sensors and try again.")
				return False

	def startDataTransmission(self):
		self.transmitData = True
		self.transmissionStartTime = rospy.Time.now()
		self.arduino.write(bytes(b'BEGIN\r\n'))

	def streamResults(self):
		if self.transmitData:
			msg = str(self.arduino.readline(), 'utf-8')
			splitMsg = msg.split(',')

			if len(splitMsg) >= 1:
				timestampNsec = int(splitMsg[0]) * 1e6 + self.transmissionStartTime.nsecs
				timestamp = rospy.Time(int(self.transmissionStartTime.secs + timestampNsec // 1e9), int(timestampNsec % 1e9))

				if len(splitMsg) >= 2:
					self.IMU.publishData(timestamp, splitMsg[1])