#!/usr/bin/env python 

'''
RBE 500 Final Project - Part A3 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 19 - Nov - 2019 

# Node Name : SCARA_check
# Subscriber Name : SCARA_T7/joint_states
# Services called : SCARA_T7_FK, SCARA_T7_IK
# Message Type : Float64MultiArray
'''

from sensor_msgs.msg import JointState
from final_project.srv import *
import numpy as np
import math as m
import rospy
np.set_printoptions(suppress=True)
counter = 0

def callback_JointState(data):
   global counter
   # This is to ensure that after processing 1 set of data, the code exits
   if counter == 0:
      counter = 1
      q_read = list(data.position)
      # Converting to degrees
      q_read[0] = q_read[0]*180/m.pi
      q_read[1] = q_read[1]*180/m.pi
      print ("Joint Positions Read (Q1) = " + str(np.around(q_read,2)))
      pose_calc = FK_service(q_read)
      print ("Calculated Pose (P1) = " + str(np.around(pose_calc.pose,2)))
      q_calc = IK_service(pose_calc.pose)
      print ("Calculated Joint Position (Q2) = " + str(np.around(q_calc.q,2)))
      if (np.array_equal(np.around(q_calc.q,6),np.around(q_read,6)) == True):
         print ("Test successful : Q1 and Q2 match\n")
      else:
         print ("Test unsuccessful : Q1 and Q2 don't match\n")
      counter = 2

# Initiating the node calling subscriber
if __name__ == "__main__":
   rospy.init_node('SCARA_T7_Check', anonymous = False)
   print("\nWaiting for the required services...")
   rospy.wait_for_service('SCARA_T7_FK')
   FK_service = rospy.ServiceProxy('SCARA_T7_FK', FK_srv)
   rospy.wait_for_service('SCARA_T7_IK')
   IK_service = rospy.ServiceProxy('SCARA_T7_IK', IK_srv)
   print("Required services up and running...")
   print("Starting subscriber")
   rospy.Subscriber("/SCARA_T7/joint_states", JointState, callback_JointState)
   while counter <= 1:
      continue
