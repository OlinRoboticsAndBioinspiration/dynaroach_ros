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

def receive_packet(payload, dynaroach, publisher):
    publisher.publish(str(payload))

def receive_time(payload, dynaroach, publisher):
    if payload.type == cmd.DATA_STREAMING:
        datum = list(unpack('<L3f3h2HB4H', payload.data))
        publisher.publish(datum[0])

def receive_gyro(payload, dynaroach, p1, p2, p3):
    if payload.type == cmd.DATA_STREAMING:
        datum = list(unpack('<L3f3h2HB4H', payload.data))
        p1.publish(datum[1])
        p2.publish(datum[2])
        p3.publish(datum[3])

def receive_accel(payload, dynaroach,  p1, p2, p3):
    if payload.type == cmd.DATA_STREAMING:
        datum = list(unpack('<L3f3h2HB4H', payload.data))
        p1.publish(datum[4])
        p2.publish(datum[5])
        p3.publish(datum[6])

def receive_backemf(payload, dynaroach, publisher):
    if payload.type == cmd.DATA_STREAMING:
        datum = list(unpack('<L3f3h2HB4H', payload.data))
        publisher.publish(datum[7])

def receive_hall(payload, dynaroach, publisher):
    if payload.type == cmd.DATA_STREAMING:
        datum = list(unpack('<L3f3h2HB4H', payload.data))
        publisher.publish(datum[9])

def receive_batt(payload, dynaroach, publisher):
    if payload.type == cmd.DATA_STREAMING:
        datum = list(unpack('<L3f3h2HB4H', payload.data))
        publisher.publish(datum[10])

def dynaroach_node(dynaroach):
    
    pub_packet = rospy.Publisher('dynaroach/packet', String)
    pub_time = rospy.Publisher('dynaroach/time', Int64)
    pub_gyro_yaw = rospy.Publisher('dynaroach/gyro/yaw', Float64)
    pub_gyro_pitch = rospy.Publisher('dynaroach/gyro/pitch', Float64)
    pub_gyro_roll = rospy.Publisher('dynaroach/gyro/roll', Float64)
    pub_accel_x = rospy.Publisher('dynaroach/accel/x', Int64)
    pub_accel_y = rospy.Publisher('dynaroach/accel/y', Int64)
    pub_accel_z = rospy.Publisher('dynaroach/accel/z', Int64)
    pub_backemf = rospy.Publisher('dynaroach/backemf', Int64)
    pub_hall = rospy.Publisher('dynaroach/hall', Bool)
    pub_batt = rospy.Publisher('dynaroach/batt', Int64)
    
    rospy.init_node('dynaroach_node', anonymous=True)
    r = rospy.Rate(.1) # 10hz
    
    dynaroach.set_data_streaming(True)
    
    dynaroach.add_receive_callback(partial(receive_packet, dynaroach=dynaroach, publisher=pub_packet))
    dynaroach.add_receive_callback(partial(receive_time, dynaroach=dynaroach, publisher=pub_time))
    dynaroach.add_receive_callback(partial(receive_gyro, dynaroach=dynaroach, p1=pub_gyro_yaw, p2=pub_gyro_pitch, p3=pub_gyro_roll))
    dynaroach.add_receive_callback(partial(receive_accel, dynaroach=dynaroach, p1=pub_accel_x, p2=pub_accel_y, p3=pub_accel_z))
    dynaroach.add_receive_callback(partial(receive_backemf, dynaroach=dynaroach, publisher=pub_backemf))
    dynaroach.add_receive_callback(partial(receive_hall, dynaroach=dynaroach, publisher=pub_hall))
    dynaroach.add_receive_callback(partial(receive_hall, dynaroach=dynaroach, publisher=pub_batt))

    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':

    dr = dr.DynaRoach(sys.argv[1])

    dynaroach_node(dr)