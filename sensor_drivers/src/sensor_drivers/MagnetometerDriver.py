#!/usr/bin/env python3

import rospy

from sensor_drivers import utils
from sensor_msgs.msg import MagneticField

class MagnetometerDriver:
    def __init__(self):
        self.seqId = 0

        self.magneticFieldVariancesMGauss = rospy.get_param('/mag/magnetic_field_variance')
        if len(self.magneticFieldVariancesMGauss) != 3:
            rospy.logerr('There must be 3 components for the magnetic field variance.')
            rospy.signal_shutdown('Error getting ROS params.')

        self.magFieldPub = rospy.Publisher('/mag', MagneticField, queue_size=1)
    
    def config(self, magRange):
        self.magRange = magRange

        self.magScaleFromRange()

        self.magneticFieldVariancesTesla = [var * 1e-7 for var in self.magneticFieldVariancesMGauss]

        self.preFillMagFieldMsg()
    
    def preFillMagFieldMsg(self):
        self.magFieldMsg = MagneticField()

        self.magFieldMsg.header.frame_id = 'mag'

        # Set magnetic field covariance
        self.magFieldMsg.magnetic_field_covariance[0] = self.magneticFieldVariancesTesla[0]
        self.magFieldMsg.magnetic_field_covariance[3] = self.magneticFieldVariancesTesla[1]
        self.magFieldMsg.magnetic_field_covariance[6] = self.magneticFieldVariancesTesla[2]
    
    def publishData(self, timestamp, rawDataStr):
        # Fill header
        self.magFieldMsg.header.seq = self.seqId
        self.magFieldMsg.header.stamp = timestamp

        # Fill magnetic field
        magField = self.magFieldFromRawData(rawDataStr)
        self.magFieldMsg.magnetic_field.x = magField[0]
        self.magFieldMsg.magnetic_field.y = magField[1]
        self.magFieldMsg.magnetic_field.z = magField[2]

        self.magFieldPub.publish(self.magFieldMsg)

        self.seqId += 1

    def magFieldFromRawData(self, rawMagFieldDataStr):
        # Get raw data from HEX string
        rawMagFieldZ = utils.hex2Dec(rawMagFieldDataStr[0:4])
        rawMagFieldY = utils.hex2Dec(rawMagFieldDataStr[4:8])
        rawMagFieldX = utils.hex2Dec(rawMagFieldDataStr[8:])

        # Get magnetic field in Tesla
        multFactor = 1.0 / (self.magScale * 1e4)
        return [rawMagFieldX * multFactor,
                rawMagFieldY * multFactor,
                rawMagFieldZ * multFactor]

    def magScaleFromRange(self):
        if self.magRange == 4:
            self.magScale = 6842.0
        elif self.magRange == 8:
            self.magScale = 3421.0
        elif self.magRange == 12:
            self.magScale = 2281.0
        elif self.magRange == 16:
            self.magScale = 1711.0