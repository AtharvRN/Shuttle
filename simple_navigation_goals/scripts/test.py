#!/usr/bin/env python3

from ast import AnnAssign
from logging.config import listen
import rospy
from std_msgs.msg import Bool,UInt8

def enable(data):
    print(data)

def waypoints(data):
    print(data)


def listen():
    print("start")
    rospy.init_node("display",anonymous=True)
    rospy.Subscriber("move_base_enable", Bool, enable)
    rospy.Subscriber("waypoint_no", UInt8, waypoints)
    
    
if __name__=='__main__':
    listen()