#!/usr/bin/env python3

import math
import rospy

from sensor_drivers import utils
from sensor_msgs.msg import Imu

class IMUDriver:
    def __init__(self):
        self.seqId = 0

        self.degToRad = math.pi / 180.0
        self.earthGravity = 9.80665

        self.linAccVarianceMg = rospy.get_param('/imu/linear_acceleration_variance')
        self.angVelVarianceDPS = rospy.get_param('/imu/angular_velocity_variance')

        self.imuPub = rospy.Publisher('/imu', Imu, queue_size=1)

    def config(self, freq, accelRange, gyroRange):
        self.freq = freq
        self.accelRange = accelRange
        self.gyroRange = gyroRange

        self.accelScaleFromRange()
        self.gyroScaleFromRange()

        self.linAccVarianceRadS2 = self.linAccVarianceMg * self.earthGravity / 1000.0
        self.angVelVarianceRadS = self.angVelVarianceDPS * self.degToRad

        self.preFillIMUMsg()

    def preFillIMUMsg(self):
        self.imuMsg = Imu()

        self.imuMsg.header.frame_id = 'imu'

        # Disable orientation
        self.imuMsg.orientation_covariance[0] = -1

        # Set angular velocity covariance
        self.imuMsg.angular_velocity_covariance[0] = self.angVelVarianceRadS
        self.imuMsg.angular_velocity_covariance[3] = self.angVelVarianceRadS
        self.imuMsg.angular_velocity_covariance[6] = self.angVelVarianceRadS

        # Set linear acceleration covariance
        self.imuMsg.linear_acceleration_covariance[0] = self.linAccVarianceRadS2
        self.imuMsg.linear_acceleration_covariance[3] = self.linAccVarianceRadS2
        self.imuMsg.linear_acceleration_covariance[6] = self.linAccVarianceRadS2
    
    def publishData(self, timestamp, rawDataStr):
        # Fill header
        self.imuMsg.header.seq = self.seqId
        self.imuMsg.header.stamp = timestamp

        # Fill angular velocity
        angVel = self.angVelFromRawData(rawDataStr[12:24])
        self.imuMsg.angular_velocity.x = angVel[0]
        self.imuMsg.angular_velocity.y = angVel[1]
        self.imuMsg.angular_velocity.z = angVel[2]

        # Fill linear acceleration
        linAcc = self.linAccFromRawData(rawDataStr[0:12])
        self.imuMsg.linear_acceleration.x = linAcc[0]
        self.imuMsg.linear_acceleration.y = linAcc[1]
        self.imuMsg.linear_acceleration.z = linAcc[2]

        self.imuPub.publish(self.imuMsg)

        self.seqId += 1
    
    def angVelFromRawData(self, rawAngVelDataStr):
        # Get raw data from HEX string
        rawAngVelZ = utils.hex2Dec(rawAngVelDataStr[0:4])
        rawAngVelY = utils.hex2Dec(rawAngVelDataStr[4:8])
        rawAngVelX = utils.hex2Dec(rawAngVelDataStr[8:])

        # Get angular velocity in rad / s^2
        multFactor = self.gyroScale * self.degToRad / 1000.0
        return [rawAngVelX * multFactor,
                rawAngVelY * multFactor,
                rawAngVelZ * multFactor]
    
    def linAccFromRawData(self, rawlinAccDataStr):
        # Get raw data from HEX string
        rawlinAccZ = utils.hex2Dec(rawlinAccDataStr[0:4])
        rawlinAccY = utils.hex2Dec(rawlinAccDataStr[4:8])
        rawlinAccX = utils.hex2Dec(rawlinAccDataStr[8:])

        # Get linear acceleration in m / s^2
        multFactor = self.accelScale * self.earthGravity / 1000.0
        return [rawlinAccX * multFactor,
                rawlinAccY * multFactor,
                rawlinAccZ * multFactor]
    
    def accelScaleFromRange(self):
        self.accelScale = 0.061 * self.accelRange / 2.0
    
    def gyroScaleFromRange(self):
        self.gyroScale = 4.375 * self.gyroRange / 125.0