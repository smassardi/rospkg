#!/usr/bin/env python

import rospy
import csv
import time

from std_msgs.msg import Bool,Int8MultiArray

class Master():

    def __init__(self, mac='', hostname='', side=''):

        #starts the master node
        rospy.init_node('master')

        # sets the two publishers of the start and test_ID
        self.pub_start = rospy.Publisher('start', Bool, queue_size = 1)
        self.pub_test_ID = rospy.Publisher('test_ID', Int8MultiArray, queue_size = 1, latch = True)
        self.rate = rospy.Rate(50)

    def start(self, cmd_start):
        msg = Bool()
        msg.data = cmd_start
        self.pub_start.publish(msg)

    def set_test_ID(self, num_subject=0, num_condition=0, num_run=0):
        ID = Int8MultiArray()
        ID.data.append(num_subject)
        ID.data.append(num_condition)
        ID.data.append(num_run)
        self.pub_test_ID.publish(ID)
