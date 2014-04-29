'''
File: dynaroach_ros.py
Author: Shivam S. Desai
Date: 2014-04-04
Description: A class representing the functionality necessary for testing and
running the dynaRoACH robot using ROS.
'''

import sys
import time
import math
from functools import partial

from serial import Serial, SerialException
import numpy as np

from struct import pack, unpack
from operator import attrgetter

sys.path.append("lib/dynaroach/python/")

from lib import cmd
import dynaroach as dr
from lib.basestation import BaseStation
from lib.payload import Payload

import rospy
from std_msgs.msg import *

import atexit

class DynaRoachNode(dr.DynaRoach):

    def __init__(self, device_name='/dev/ttyUSB0'):

        super(DynaRoachNode, self).__init__(device_name)

        self.pub_packet = rospy.Publisher('dynaroach/packet', String)
        self.pub_time = rospy.Publisher('dynaroach/time', UInt32)
        self.pub_gyro_yaw = rospy.Publisher('dynaroach/gyro/yaw', Float32)
        self.pub_gyro_pitch = rospy.Publisher('dynaroach/gyro/pitch', Float32)
        self.pub_gyro_roll = rospy.Publisher('dynaroach/gyro/roll', Float32)
        self.pub_accel_x = rospy.Publisher('dynaroach/accel/x', UInt16)
        self.pub_accel_y = rospy.Publisher('dynaroach/accel/y', UInt16)
        self.pub_accel_z = rospy.Publisher('dynaroach/accel/z', UInt16)
        self.pub_backemf = rospy.Publisher('dynaroach/backemf', UInt16)
        self.pub_hall = rospy.Publisher('dynaroach/hall', Bool)
        self.pub_batt = rospy.Publisher('dynaroach/batt', UInt16)

        rospy.init_node('dynaroach_node', anonymous=True)

        self.set_data_streaming(True)

        self.add_receive_callback(self.receive_packet)
        self.add_receive_callback(self.receive_time)
        self.add_receive_callback(self.receive_gyro)
        self.add_receive_callback(self.receive_accel)
        self.add_receive_callback(self.receive_backemf)
        self.add_receive_callback(self.receive_hall)
        self.add_receive_callback(self.receive_batt)

    def receive_packet(self, payload):
        self.pub_packet.publish(str(payload))

    def receive_time(self, payload):
        if payload.type == cmd.DATA_STREAMING:
            datum = list(unpack('<L3f3h2HB4H', payload.data))
            self.pub_time.publish(datum[0])

    def receive_gyro(self, payload):
        if payload.type == cmd.DATA_STREAMING:
            datum = list(unpack('<L3f3h2HB4H', payload.data))
            self.pub_gyro_yaw.publish(datum[1])
            self.pub_gyro_pitch.publish(datum[2])
            self.pub_gyro_roll.publish(datum[3])

    def receive_accel(self, payload):
        if payload.type == cmd.DATA_STREAMING:
            datum = list(unpack('<L3f3h2HB4H', payload.data))
            self.pub_accel_x.publish(datum[4])
            self.pub_accel_y.publish(datum[5])
            self.pub_accel_z.publish(datum[6])

    def receive_backemf(self, payload):
        if payload.type == cmd.DATA_STREAMING:
            datum = list(unpack('<L3f3h2HB4H', payload.data))
            self.pub_backemf.publish(datum[7])

    def receive_hall(self, payload):
        if payload.type == cmd.DATA_STREAMING:
            datum = list(unpack('<L3f3h2HB4H', payload.data))
            self.pub_hall.publish(datum[9])

    def receive_batt(self, payload):
        if payload.type == cmd.DATA_STREAMING:
            datum = list(unpack('<L3f3h2HB4H', payload.data))
            self.pub_batt.publish(datum[10])

    def run(self):
        r = rospy.Rate(.1) # 10hz
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':

    dyn = DynaRoachNode(sys.argv[1])

    dyn.run()