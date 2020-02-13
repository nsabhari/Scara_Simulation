#!/usr/bin/env python 

'''
RBE 500 Final Project - Part A1 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 19 - Nov - 2019 

# Server(Node) Name : SCARA_T7_FK_Server
# Service Name : SCARA_T7_FK (Service Type Name : FK_srv)
'''

from final_project.srv import *
import rospy
import numpy as np
import math as m

# a, alpha, d, theta : 99 means variable
DH_param = [[0.2 ,0.0 ,0.2     ,99],
            [0.2 ,180 ,0.0     ,99],
            [0.0 ,0.0 ,0.1+99  ,0 ]]
DH_A = np.zeros([3, 4, 4], dtype=float)
DH_T = np.zeros([3, 4, 4], dtype=float)
Euler = [0.0, 0.0, 0.0]
np.set_printoptions(suppress=True)

# Function to calculate transformation matrix
def Calc_Matrix_A_T(q1,q2,q3):
   global DH_param, DH_A, DH_T
                                        
   # Update DH Table
   DH_param[0][3] = q1
   DH_param[1][3] = q2
   DH_param[2][2] = 0.1+q3

   # Calculate A1, A2, A3
   for i in range(0,3):
      DH_A[i][0][0] = np.cos(np.radians(DH_param[i][3]))                                        # cos(theta)
      DH_A[i][0][1] = -np.sin(np.radians(DH_param[i][3]))*np.cos(np.radians(DH_param[i][1]))    #-sin(theta)*cos(alpha)
      DH_A[i][0][2] = np.sin(np.radians(DH_param[i][3]))*np.sin(np.radians(DH_param[i][1]))     # sin(theta)*sin(alpha)
      DH_A[i][0][3] = DH_param[i][0]*np.cos(np.radians(DH_param[i][3]))                         # a*cos(theta)

      DH_A[i][1][0] = np.sin(np.radians(DH_param[i][3]))                                        # sin(theta)
      DH_A[i][1][1] = np.cos(np.radians(DH_param[i][3]))*np.cos(np.radians(DH_param[i][1]))     # cos(theta)*cos(alpha)
      DH_A[i][1][2] = -np.cos(np.radians(DH_param[i][3]))*np.sin(np.radians(DH_param[i][1]))    #-cos(theta)*sin(alpha)
      DH_A[i][1][3] = DH_param[i][0]*np.sin(np.radians(DH_param[i][3]))                         # a*sin(theta)

      DH_A[i][2][0] = 0                                                                         # 0      
      DH_A[i][2][1] = np.sin(np.radians(DH_param[i][1]))                                        # sin(alpha)
      DH_A[i][2][2] = np.cos(np.radians(DH_param[i][1]))                                        # cos(alpha)
      DH_A[i][2][3] = DH_param[i][2]                                                            # d

      DH_A[i][3][0] = 0                                                                         # 0
      DH_A[i][3][1] = 0                                                                         # 0      
      DH_A[i][3][2] = 0                                                                         # 0      
      DH_A[i][3][3] = 1  

   # Calculating the transformation matrices
   DH_T[0] = DH_A[0]
   DH_T[1] = np.dot(DH_T[0],DH_A[1])
   DH_T[2] = np.dot(DH_T[1],DH_A[2])
   
# Function to calculate FK & return the pose
def handle_SCARA_FK(data):
   global Euler
   print ("\nFORWARD KINEMATICS CALCULATOR SERVICE CALLED")
   print ("Input : q (q1,q2,q3) = " + str(np.around(data.q,4)))
   Calc_Matrix_A_T(data.q[0],data.q[1],data.q[2])
   
   # Caluculating Euler Angles (Z0,Y,Z1)
   Rot_Mat = DH_T[2,0:3,0:3]
   if Rot_Mat[2,2] < 1:    
      if Rot_Mat[2,2] > -1:   # Cos(Y) = (-1,+1)
         Euler[1] = m.acos(Rot_Mat[2,2])
         Euler[0] = m.atan2(Rot_Mat[1,2],Rot_Mat[0,2])
         Euler[2] = m.atan2(Rot_Mat[2,1],-Rot_Mat[2,0])
      else:                   # Cos(Y) = -1
         Euler[1] = m.pi
         Euler[0] = 0
         Euler[2] = m.atan2(Rot_Mat[1,0],Rot_Mat[1,1])
   else:                      # Cos(Y) = +1
      Euler[1] = 0
      Euler[0] = 0
      Euler[2] = m.atan2(Rot_Mat[1,0],Rot_Mat[1,1])
      
   Euler = np.multiply(Euler,180/m.pi)
   pose = np.concatenate((DH_T[2,0:3,3],Euler),axis=None)
   print ("Output : Pose (x,y,z,Z0,Y,Z1) = " + str(np.around(pose,4)))
   return FK_srvResponse(pose)

# Function to initiate the services and wait for calls
def SCARA_server():
   rospy.init_node('SCARA_T7_FK_Server')
   srv_1 = rospy.Service('SCARA_T7_FK', FK_srv, handle_SCARA_FK)
   print "Ready to calculate Forward Kinematics for SCARA."
   rospy.spin()

# Call function to intiate the service
if __name__ == "__main__":
     SCARA_server()
