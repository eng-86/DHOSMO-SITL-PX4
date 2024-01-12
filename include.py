#!/usr/bin/env python
from __future__ import division
import sys
import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ParamValue, PositionTarget, Mavlink, State, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode
from vehicle_data.msg import Reconstruction, ObserverError
from tf.transformations import euler_from_quaternion

class initialize:

    def __init__(self,id):

        self.pos = np.array([0.0, 0.0, 0.0]) 
        self.vel = np.array([0.0, 0.0, 0.0])
        self.att = np.array([0.0, 0.0, 0.0])
        self.pos_sp = np.array([0.0, 0.0, 0.0])
        self.vel_sp = np.array([0.0, 0.0, 0.0])
        self.omega = np.array([0.0, 0.0, 0.0])
        self.controls = np.array([0.0, 0.0, 0.0, 0.0])
        self.error = np.array([0.0, 0.0, 0.0, 0.0])
        self.desired_traj = PositionTarget()
        self.reconstructed = Reconstruction()
        self.current_error = ObserverError()
        self.ct = 0.0
        self.ctatt = 0.0
        self.ctcont = 0.0

        #initialize subscribers
        self.state_sub = rospy.Subscriber('uav'+str(id)+'/mavros/state', State, self.state_cb)
        self.position_sub = rospy.Subscriber('uav'+str(id)+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        self.local_vel_sub = rospy.Subscriber('uav'+str(id)+'/mavros/local_position/velocity_local', TwistStamped, self.local_velocity_cb)
        self.pos_setpoint_sub = rospy.Subscriber('uav'+str(id)+'/mavros/setpoint_raw/local', PositionTarget, self.desired_traj_cb)
        self.act_control_sub = rospy.Subscriber('uav'+str(id)+'/mavros/target_actuator_control', ActuatorControl, self.actuator_cnt_cb)
        self.obs_error_sub = rospy.Subscriber('uav'+str(id)+'/som/observer_error', ObserverError, self.state_error_cb)

	#initialize publishers
        self.pos_setpoint_pub = rospy.Publisher('uav'+str(id)+'/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.reconstruct_fault_pub = rospy.Publisher('uav'+str(id)+'/som/reconstruction', Reconstruction, queue_size=10)
        self.error_fault_pub = rospy.Publisher('uav'+str(id)+'/som/observer_error', ObserverError, queue_size=10)

        #initialize services
        self.arming_client = rospy.ServiceProxy('uav'+str(id)+'/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('uav'+str(id)+'/mavros/set_mode', SetMode)

    #callback for vehicle's state subscriber
    def state_cb(self, message):
        self.current_state = message
        self.current_state = State()

    #callback for vehicle's local position subscriber
    def local_position_cb(self, message):
        self.pos = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
        self.att = euler_from_quaternion([message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w])
        self.ctatt = (message.header.stamp.nsecs)*1e-9

    #callback for vehicle's local linear and angular velocity subscriber
    def local_velocity_cb(self, message):
        self.vel = [message.twist.linear.x, message.twist.linear.y, message.twist.linear.z]
        self.omega = [message.twist.angular.x, message.twist.angular.y, message.twist.angular.z]
        self.ct = (message.header.stamp.nsecs)*1e-9

    #callback for vehicle's desired trajectory subscriber
    def desired_traj_cb(self, message):
        self.pos_sp = [message.position.x, message.position.y, message.position.z]
        self.vel_sp = [message.velocity.x, message.velocity.y, message.velocity.z]
    
    #callback for vehicle's actuator control subscriber 
    def actuator_cnt_cb(self, message):
        self.controls = np.array([message.controls[0], message.controls[1], message.controls[2], message.controls[3]])
        self.ctcont = (message.header.stamp.nsecs)*1e-9

    #callback for vehicle's observer fault reconstruction subscriber 
    def state_error_cb(self, message):
        self.error = np.array([message.error[0], message.error[1], message.error[2], message.error[3]])

    #function to return the saturation of an integer
    def sat(self, input_int, gain):
        if abs(input_int) < gain:
            sat_value = input_int
        else:
            sat_value = math.copysign(1.0, input_int)
        return sat_value
        

    #function that returns true in case the vehicle reaches its desired position
    def is_at_position(self, desired_x, desired_y, desired_z, current_x, current_y, current_z, offset):
        desired = np.array((desired_x, desired_y, desired_z))
        pos = np.array((current_x,
                        current_y,
                        current_z))
        return np.linalg.norm(desired - pos) < offset
