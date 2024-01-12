#!/usr/bin/env python

import rospy
import sys
import math
import numpy as np
from include import initialize
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header
from vehicle_data.msg import Reconstruction, ObserverError
import rosbag

# Constants for formation control
NEIGHBOR_DISTANCE_THRESHOLD = 5.0  # Threshold distance for neighbor detection
uav_num = 3 # Number of UAVs in the formation

reconstructed0 = Reconstruction()
reconstructed1 = Reconstruction()
reconstructed2 = Reconstruction()
reconstructed3 = Reconstruction()


#callback for vehicle's reconstruction subscriber
def reconstruction_cb_0(message):
    reconstructed0.vroll=message.vroll
    reconstructed0.vpitch=message.vpitch
    reconstructed0.vyaw=message.vyaw
    reconstructed0.vz=message.vz
    reconstructed0.phi=message.phi
    reconstructed0.theta=message.theta
    reconstructed0.psi=message.psi
    reconstructed0.z=message.z

def reconstruction_cb_1(message):
    reconstructed1.vroll=message.vroll
    reconstructed1.vpitch=message.vpitch
    reconstructed1.vyaw=message.vyaw
    reconstructed1.vz=message.vz
    reconstructed1.phi=message.phi
    reconstructed1.theta=message.theta
    reconstructed1.psi=message.psi
    reconstructed1.z=message.z

def reconstruction_cb_2(message):
    reconstructed2.vroll=message.vroll
    reconstructed2.vpitch=message.vpitch
    reconstructed2.vyaw=message.vyaw
    reconstructed2.vz=message.vz
    reconstructed2.phi=message.phi
    reconstructed2.theta=message.theta
    reconstructed2.psi=message.psi
    reconstructed2.z=message.z

def reconstruction_cb_3(message):
    reconstructed3.vroll=message.vroll
    reconstructed3.vpitch=message.vpitch
    reconstructed3.vyaw=message.vyaw
    reconstructed3.vz=message.vz
    reconstructed3.phi=message.phi
    reconstructed3.theta=message.theta
    reconstructed3.psi=message.psi
    reconstructed3.z=message.z


bag = rosbag.Bag('test.bag', 'w')

if __name__ == '__main__':


    # Initialization
    rospy.init_node('log_node') 

    rospy.Subscriber('uav0/som/reconstruction', Reconstruction, reconstruction_cb_0)
    rospy.Subscriber('uav1/som/reconstruction', Reconstruction, reconstruction_cb_1)
    rospy.Subscriber('uav2/som/reconstruction', Reconstruction, reconstruction_cb_2)
    rospy.Subscriber('uav3/som/reconstruction', Reconstruction, reconstruction_cb_3)


    rate = rospy.Rate(100)  # Loop speed 100Hz	
    while not rospy.is_shutdown():
        bag.write('uav0', reconstructed0)
        bag.write('uav1', reconstructed1)
        bag.write('uav2', reconstructed2)
        bag.write('uav3', reconstructed3)
        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            bag.close()
            pass
    
    # Spin the node
    rospy.spin()
