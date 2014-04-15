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

from serial import Serial, SerialException
import numpy as np

from struct import pack, unpack
from operator import attrgetter

from lib import cmd
from lib import dynaroach as dr
from lib.basestation import BaseStation
from lib.payload import Payload

import rospy
from std_msgs.msg import String

def dynaroach_node(dynaroach):
    pub = rospy.Publisher('dynaroach/time', String)
    rospy.init_node('dynaroach_node', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        dynaroach.get_gyro_calib_param()
        
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':

    dr = dr.DynaRoach(sys.argv[1])

    dynaroach_node(dr)