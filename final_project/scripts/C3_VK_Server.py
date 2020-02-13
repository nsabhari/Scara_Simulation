#!/usr/bin/env python 

'''
RBE 500 Final Project - Part C3 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 13 - Dec - 2019 

# Server(Node) Name : SCARA_T7_VK_Server
# Service Name : SCARA_T7_VK (Service Type Name : VK_srv)
# Service Name : SCARA_T7_IVK (Service Type Name : VK_srv)
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
Jacobian = np.zeros([6, 3], dtype=float)

np.set_printoptions(suppress=True)

# a, alpha, d, theta : 99 means variable
DH_param = [[0.2 ,0.0 ,0.2     ,99],
            [0.2 ,180 ,0.0     ,99],
            [0.0 ,0.0 ,0.1+99  ,0 ]]
DH_A = np.zeros([3, 4, 4], dtype=float)
DH_T = np.zeros([3, 4, 4], dtype=float)

# Function to calculate transformation matrix
def Calc_Matrix_A_T(q1,q2,q3):
   global DH_param, DH_A, DH_T
   #print ("Input (q1, q2, q3) = (" + str(q1) + ". " + str(q2) + ". " + str(q3) + "}")
                                        
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
   DH_T[0] = np.around(DH_A[0],4)
   DH_T[1] = np.around(np.dot(DH_T[0],DH_A[1]),4)
   DH_T[2] = np.around(np.dot(DH_T[1],DH_A[2]),4)
   
def handle_SCARA_FK(data):
   Calc_Matrix_A_T(data.q[0],data.q[1],data.q[2])
   return FK_srvResponse(DH_T[2,0:3,3])
   
def handle_SCARA_VK(data):
   global Jacobian
   #print ("\nJACOBIAN CALCULATOR SERVICE CALLED")
   Calc_Matrix_A_T(data.input[0],data.input[1],data.input[2])
   Q_dot=[[data.input[3]],[data.input[4]],[data.input[5]]]
   
   Jacobian[0:3,0] = np.cross([0,0,1],DH_T[2,0:3,3])
   Jacobian[3:6,0] = [0,0,1]
   Jacobian[0:3,1] = np.cross(DH_T[0,0:3,2],(DH_T[2,0:3,3]-DH_T[0,0:3,3]))
   Jacobian[3:6,1] = DH_T[0,0:3,2]
   Jacobian[0:3,2] = DH_T[1,0:3,2]
   Jacobian[3:6,2] = [0,0,0]
   
   Velocity = np.dot(Jacobian, Q_dot)

   #print ("End Effector Velocities = \n" + str(Velocity))
   return VK_srvResponse(Velocity.ravel())

def handle_SCARA_IVK(data):
   global Jacobian
   #print ("\nJACOBIAN CALCULATOR SERVICE CALLED")
   Calc_Matrix_A_T(data.input[0],data.input[1],data.input[2])
   Velocity=[[data.input[3]],[data.input[4]],[data.input[5]]]
   
   Jacobian[0:3,0] = np.cross([0,0,1],DH_T[2,0:3,3])
   Jacobian[3:6,0] = [0,0,1]
   Jacobian[0:3,1] = np.cross(DH_T[0,0:3,2],(DH_T[2,0:3,3]-DH_T[0,0:3,3]))
   Jacobian[3:6,1] = DH_T[0,0:3,2]
   Jacobian[0:3,2] = DH_T[1,0:3,2]
   Jacobian[3:6,2] = [0,0,0]
   Jacobian_Inv = np.linalg.inv(Jacobian[0:3,0:3])
   
   J_dot=np.dot(Jacobian_Inv, Velocity)

   #print ("Joint Velocity = \n" + str(J_dot.ravel()))
   return VK_srvResponse(J_dot.ravel())

# Function to initiate the services and wait for calls
def SCARA_server():
   rospy.init_node('SCARA_T7_VK_Server')
   srv1 = rospy.Service('SCARA_T7_VK', VK_srv, handle_SCARA_VK)
   print "Ready to calculate Velocities Kinematics for SCARA."
   srv2 = rospy.Service('SCARA_T7_IVK', VK_srv, handle_SCARA_IVK)
   print "Ready to calculate Inverse Velocity Kinematics for SCARA."
   srv3 = rospy.Service('SCARA_T7_FK', FK_srv, handle_SCARA_FK)
   print "Ready to calculate Forward Kinematics for SCARA."
   rospy.spin()

# Call function to intiate the service
if __name__ == "__main__":
     SCARA_server()
