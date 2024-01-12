#!/usr/bin/env python

import rospy
import sys
import math
import numpy as np
from include import initialize
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header
from vehicle_data.msg import Reconstruction, ObserverError

# Constants for formation control
NEIGHBOR_DISTANCE_THRESHOLD = 5.0  # Threshold distance for neighbor detection
uav_num = 4 # Number of UAVs in the formation
initial_formation = np.array([[0,0,3],[4,0,3],[4,4,3],[0,4,3]])

hover = 0 # Hovering flag

uavn = [initialize(i) for i in range(uav_num)] # Initialize the topics 

uav_id =  int(sys.argv[1]) # UAV id
uav = initialize(uav_id) # Current UAV initializer
set_mode = uav.set_mode_client() # Setting mode service
arm_cmd = uav.arming_client() # Arming service

# Defining the target position message
uav.desired_traj.header = Header()
uav.desired_traj.header.frame_id = "base_footprint"
uav.desired_traj.coordinate_frame = 1
uav.desired_traj.type_mask = 0

# Observer states
x1hatd = np.zeros(4)
x1hat = np.zeros(4)
x2hatd = np.zeros(4)
x2hat = np.zeros(4) 
zeqd = np.zeros(4)
zeq = np.zeros(4) 
y1 = np.zeros((uav_num, 4))
y1hatd = np.zeros((uav_num, 4))
y1hat = np.zeros((uav_num, 4))
y2hatd = np.zeros((uav_num, 4))
y2hat = np.zeros((uav_num, 4))
weqd = np.zeros((uav_num, 4))
weq = np.zeros((uav_num, 4))

# HOSMO reconstruction parameters 
B = np.array([34.33, 34.33, 18.1, 1]) 
dI = 0.896 # (Izz - Ixx)/Iyy

alfa0 = [1.1*5.0,1.1*5.0,1.1*2.5,1.1*0.14]
alfa1 = [1.5*pow(5.0,1/2),1.5*pow(5.0,1/2),1.5*pow(2.5,1/2),1.5*pow(0.14,1/2)]
alfa2 = [1.9*pow(5.0,1/3),1.9*pow(5.0,1/3),1.9*pow(2.5,1/3),1.9*pow(0.14,1/3)]


# DHOSMO function

def fault_reconstruction(dt):
    
    global x1hat, x1hatd, x2hat, x2hatd, zeq, zeqd, y1hat, y1hatd, y2hat, y2hatd, weq, weqd
    
    # Compute the local observers
    x1 = np.array([uav.att[0], uav.att[1], uav.att[2], uav.pos[2]])

    uav.current_error = uav.controls

    # Publish the relative measurement message
    uav.error_fault_pub.publish(uav.current_error)


    for i in range(4):
        x1hatd[i] = x2hat[i] + alfa2[i]*pow(abs(x1[i] - x1hat[i]),2/3)*np.sign(x1[i] - x1hat[i])
    x2hatd[0] = B[0]*uav.controls[0] - dI*x2hat[1]*x2hat[2] + alfa1[0]*pow(abs(x1hatd[0] - x2hat[0]),1/2)*np.sign(x1hatd[0] - x2hat[0]) + zeq[0]
    x2hatd[1] = B[1]*uav.controls[1] + dI*x2hat[0]*x2hat[2] + alfa1[1]*pow(abs(x1hatd[1] - x2hat[1]),1/2)*np.sign(x1hatd[1] - x2hat[1]) + zeq[1]
    x2hatd[2] = B[2]*uav.controls[2] + alfa1[2]*pow(abs(x1hatd[2] - x2hat[2]),1/2)*np.sign(x1hatd[2] - x2hat[2]) + zeq[2]
    x2hatd[3] = B[3]*uav.controls[3] + alfa1[3]*pow(abs(x1hatd[3] - x2hat[3]),1/2)*np.sign(x1hatd[3] - x2hat[3]) + zeq[3] - 0.707
    x2hat += x2hatd*dt;
    for i in range(4):
        zeqd[i] = alfa0[i]*np.sign(x1hatd[i] - x2hat[i])

    x1hat += x1hatd*dt
    zeq += zeqd*dt

    # Check the neighbors of current UAV and compute consensus sliding mode term
    for i in range(uav_num):
        # Compute the distributed observers
        if i != uav_id:
            if np.linalg.norm((uav.pos + initial_formation[uav_id]) - (uavn[i].pos + initial_formation[i]))<NEIGHBOR_DISTANCE_THRESHOLD:
                #y1 = uavn[i].error
                y1[i,:] = np.array([uavn[i].att[0], uavn[i].att[1], uavn[i].att[2], uavn[i].pos[2]])

                for j in range(4):
                    y1hatd[i,j] = y2hat[i,j] + alfa2[j]*pow(abs(y1[i,j] - y1hat[i,j]),2/3)*np.sign(y1[i,j] - y1hat[i,j])
                y2hatd[i,0] = (B[0]*uavn[i].error[0]) - dI*y2hat[i,1]*y2hat[i,2] + alfa1[0]*pow(abs(y1hatd[i,0] - y2hat[i,0]),1/2)*np.sign(y1hatd[i,0] - y2hat[i,0]) + weq[i,0]
                y2hatd[i,1] = (B[1]*uavn[i].error[1]) + dI*y2hat[i,0]*y2hat[i,2] + alfa1[1]*pow(abs(y1hatd[i,1] - y2hat[i,1]),1/2)*np.sign(y1hatd[i,1] - y2hat[i,1]) + weq[i,1]
                y2hatd[i,2] = (B[2]*uavn[i].error[2]) + alfa1[2]*pow(abs(y1hatd[i,2] - y2hat[i,2]),1/2)*np.sign(y1hatd[i,2] - y2hat[i,2]) + weq[i,2]
                y2hatd[i,3] = (B[3]*uavn[i].error[3]) + alfa1[3]*pow(abs(y1hatd[i,3] - y2hat[i,3]),1/2)*np.sign(y1hatd[i,3] - y2hat[i,3]) + weq[i,3] - 0.707
                y2hat[i,:] += y2hatd[i,:]*dt                
                for k in range(4):
                    weqd[i,k] = alfa0[k]*np.sign(y1hatd[i,k] - y2hat[i,k])
                y1hat[i,:] += y1hatd[i,:]*dt
                weq[i,:] += weqd[i,:]*dt
                uav.reconstructed.vroll[i] = weq[i,0]/B[0]
                uav.reconstructed.vpitch[i] = weq[i,1]/B[1]
                uav.reconstructed.vyaw[i] = weq[i,2]/B[2]
                uav.reconstructed.vz[i] = weq[i,3]/B[3]
                uav.reconstructed.phi[i] = x1hat[0] - y1hat[i,0]
                uav.reconstructed.theta[i] = x1hat[1] - y1hat[i,1]
                uav.reconstructed.psi[i] = x1hat[2] - y1hat[i,2]
                uav.reconstructed.z[i] = x1hat[3] - y1hat[i,3]
                uav.reconstructed.phid[i] = x2hat[0] - y2hat[i,0]
                uav.reconstructed.thetad[i] = x2hat[1] - y2hat[i,1]
                uav.reconstructed.psid[i] = x2hat[2] - y2hat[i,2]
                uav.reconstructed.zd[i] = x2hat[3] - y2hat[i,3]
                rospy.loginfo("vehicle%d has a neighbor %d", uav_id, i,)

            else:
                uav.reconstructed.vroll[i] = 0.0
                uav.reconstructed.vpitch[i] = 0.0
                uav.reconstructed.vyaw[i] = 0.0
                uav.reconstructed.vz[i] = 0.0
                uav.reconstructed.phi[i] = 0.0
                uav.reconstructed.theta[i] = 0.0
                uav.reconstructed.psi[i] = 0.0
                uav.reconstructed.z[i] = 0.0
                uav.reconstructed.phid[i] = 0.0
                uav.reconstructed.thetad[i] = 0.0
                uav.reconstructed.psid[i] = 0.0
                uav.reconstructed.zd[i] = 0.0
        else:
            # Compute the local fault reconstrucion
            uav.reconstructed.vroll[i] = zeq[0]/B[0]
            uav.reconstructed.vpitch[i] = zeq[1]/B[1]
            uav.reconstructed.vyaw[i] = zeq[2]/B[2]
            uav.reconstructed.vz[i] = zeq[3]/B[3]
            uav.reconstructed.phi[i] = -uav.controls[0]
            uav.reconstructed.theta[i] = -uav.controls[1]
            uav.reconstructed.psi[i] = -uav.controls[2]
            uav.reconstructed.z[i] = -uav.controls[3]
            uav.reconstructed.phid[i] = x2hat[0]
            uav.reconstructed.thetad[i] = x2hat[1]
            uav.reconstructed.psid[i] = x2hat[2]
            uav.reconstructed.zd[i] = x2hat[3]

    # Publish the reconstruction message
    uav.reconstruct_fault_pub.publish(uav.reconstructed)

     

if __name__ == '__main__':

    # Initialization
    rospy.init_node('quadrotor_controller'+str(uav_id)) 
    rospy.loginfo("1. UAVs are initialized")

    rospy.sleep(5)

    rate = rospy.Rate(100)  # Loop speed 100Hz	

    # Building an initial desired set points
    uav.desired_traj.header.stamp = rospy.Time.now()
    uav.desired_traj.position.x = 0
    uav.desired_traj.position.y = 0 
    uav.desired_traj.position.z = 0
    uav.desired_traj.velocity.x = 0
    uav.desired_traj.velocity.y = 0
    uav.desired_traj.velocity.z = 0
    uav.desired_traj.acceleration_or_force.x = 0 
    uav.desired_traj.acceleration_or_force.y = 0
    uav.desired_traj.acceleration_or_force.z = 0

    # Publishing zero setpoints before OFFBOARD flight mode to avoid failsafe
    for i in range(50):
        uav.pos_setpoint_pub.publish(uav.desired_traj)
        rate.sleep()

    # Setting OFFBOARD flight mode
    set_mode = uav.set_mode_client(0, "OFFBOARD")  # 0 is custom mode
    while not set_mode:
        rospy.loginfo("Failed to send OFFBOARD mode command for vehicle%d", uav_id)
        set_mode = uav.set_mode_client(0, "OFFBOARD")  # 0 is custom mode
    
    # Arm the motors
    arm_cmd = uav.arming_client(True)
    if not arm_cmd.success:
        rospy.loginfo("vehicle%d failed to send arm command", uav_id)

    # Starting the mission
    rospy.loginfo("2. Vehicle%d is ready to fly", uav_id)
    
    # initialize time
    initial_time1 = rospy.get_time() 
    initial_time = rospy.get_time() 
    prev_time = rospy.get_time() 

    while not rospy.is_shutdown():

	# Hovering loop
	while uav.pos[2]<3 and hover==0 or (initial_time - initial_time1) < 20:
            pos_sp = np.array([0, 0, 3])
            vel_sp = np.array([0, 0, 0])
            acc_sp = np.array([0, 0, 0])
            uav.desired_traj.header.stamp = rospy.Time.now()            
            [uav.desired_traj.position.x, uav.desired_traj.position.y, uav.desired_traj.position.z] = pos_sp
            [uav.desired_traj.velocity.x, uav.desired_traj.velocity.y, uav.desired_traj.velocity.z]  = vel_sp
            [uav.desired_traj.acceleration_or_force.x, uav.desired_traj.acceleration_or_force.y, uav.desired_traj.acceleration_or_force.z] = acc_sp
            uav.desired_traj.header.stamp = rospy.Time.now() 
            uav.pos_setpoint_pub.publish(uav.desired_traj)
            initial_time = rospy.get_time()
            prev_time = rospy.get_time() 
        
        hover = 1
                  
        current_time = rospy.get_time()
        dt = current_time - prev_time
        prev_time = current_time   
        current_time = rospy.get_time()
        t = current_time - initial_time

        if (t>5):
            fault_reconstruction(dt)

        current_time = rospy.get_time()

        [uav.desired_traj.position.x, uav.desired_traj.position.y, uav.desired_traj.position.z] = pos_sp
        [uav.desired_traj.velocity.x, uav.desired_traj.velocity.y, uav.desired_traj.velocity.z]  = vel_sp
        [uav.desired_traj.acceleration_or_force.x, uav.desired_traj.acceleration_or_force.y, uav.desired_traj.acceleration_or_force.z] = acc_sp
        uav.desired_traj.header.stamp = rospy.Time.now()      
        uav.pos_setpoint_pub.publish(uav.desired_traj)

        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    # Spin the node
    rospy.spin()
