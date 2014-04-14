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
from lib import dynaRoACH as dr
from lib.basestation import BaseStation
from lib.payload import Payload

if __name__ == '__main__':
    d = dr.DynaRoach(sys.argv[1])
    d.echo()