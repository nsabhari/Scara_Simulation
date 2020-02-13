#!/usr/bin/env python 

'''
RBE 500 Final Project - Part A2 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 19 - Nov - 2019 

# Server(Node) Name : SCARA_T7_IK_Server
# Service Name : SCARA_T7_IK (Service Type Name : IK_srv)
'''

from final_project.srv import *
import rospy
import numpy as np
import math as m

# a, alpha, d, theta : 99 means variable
DH_param = [[0.2 ,0.0 ,0.2     ,99],
            [0.2 ,180 ,0.0     ,99],
            [0.0 ,0.0 ,0.1+99  ,0 ]]
np.set_printoptions(suppress=True)

# Function to calculate IK & return the possible set of "q's"
def handle_SCARA_IK(data):
   print ("\nINVERSE KINEMATICS CALCULATOR SERVICE CALLED")
   print ("Input : Pose (x,y,z,Z0,Y,Z1) = " + str(np.around(data.pose,4)))
   q3 = 0.1 - data.pose[2]             #q3 = 0.1 - z
   cos_q2 = (m.pow(data.pose[0],2)+m.pow(data.pose[1],2)-m.pow(DH_param[0][0],2)-m.pow(DH_param[1][0],2))/(2*DH_param[0][0]*DH_param[1][0])
   sin_q2_1 = m.sqrt(1-m.pow(cos_q2,2))
   sin_q2_2 = -m.sqrt(1-m.pow(cos_q2,2))
   q2_1 = m.atan2(sin_q2_1,cos_q2)     #q2 - Solution 1
   q1_1 = m.atan2(data.pose[1],data.pose[0]) - m.atan2(DH_param[1][0]*sin_q2_1,DH_param[0][0]+DH_param[1][0]*cos_q2)    #q1 - Solution 1
   q2_2 = m.atan2(sin_q2_2,cos_q2)     #q2 - Solution 2
   q1_2 = m.atan2(data.pose[1],data.pose[0]) - m.atan2(DH_param[1][0]*sin_q2_2,DH_param[0][0]+DH_param[1][0]*cos_q2)    #q1 - Solution 2

   # Selecting the one between Solution 1 & Solution 2 based on angle of x-axis of Frame 3 w.r.t x-axis of Frame 0
   Angle_frm_q_set1 = np.mod(q1_1*180/m.pi + q2_1*180/m.pi,360)
   Angle_frm_q_set2 = np.mod(q1_2*180/m.pi + q2_2*180/m.pi,360)
   Angle_frm_pose = np.mod(180 - data.pose[5],360)

   # Choosing the one in which "Angle from q is closest to Angle from pose". In ideal case (i.e. without rounding off) this should be 0
   temp_1 = m.fabs(Angle_frm_pose-Angle_frm_q_set1)
   temp_2 = m.fabs(Angle_frm_pose-Angle_frm_q_set2)
   if temp_1 < temp_2:
      q = [q1_1*180/m.pi, q2_1*180/m.pi, q3]
   else:
      q = [q1_2*180/m.pi, q2_2*180/m.pi, q3]
   print ("Output : q (q1,q2,q3) = " + str(np.around(q,4)))
   return IK_srvResponse(q)

# Function to initiate the services and wait for calls
def SCARA_server():
   rospy.init_node('SCARA_T7_IK_Server')
   srv = rospy.Service('SCARA_T7_IK', IK_srv, handle_SCARA_IK)
   print "Ready to calculate Inverse Kinematics for SCARA."
   rospy.spin()

# Call function to intiate the service
if __name__ == "__main__":
     SCARA_server()
