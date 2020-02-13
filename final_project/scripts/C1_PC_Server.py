#!/usr/bin/env python 

'''
RBE 500 Final Project - Part C1 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 13 - Dec - 2019 

# Server(Node) Name : SCARA_T7_Ref_Server
# Service Name : SCARA_T7/Joint1_Pos_Ref (Service Type Name : PC_srv)
# Service Name : SCARA_T7/Joint2_Pos_Ref (Service Type Name : PC_srv)
# Service Name : SCARA_T7/Joint3_Pos_Ref (Service Type Name : PC_srv)
# Service Name : SCARA_T7/Joint1_Vel_Ref (Service Type Name : PC_srv)
# Service Name : SCARA_T7/Joint2_Vel_Ref (Service Type Name : PC_srv)
# Service Name : SCARA_T7/Joint3_Vel_Ref (Service Type Name : PC_srv)
'''

from final_project.srv import *
import rospy
from std_msgs.msg import Float64
import numpy as np
np.set_printoptions(suppress=True)

def handle_J1P(data):
   pub = rospy.Publisher('/SCARA_T7/joint1_position_controller/command', Float64, queue_size=1)
   print ("\nJOINT 1 POSITION CONTROLLER SERVICE CALLED")
   print ("Position received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 1 position set")

def handle_J2P(data):
   pub = rospy.Publisher('/SCARA_T7/joint2_position_controller/command', Float64, queue_size=1)
   print ("\nJOINT 2 POSITION CONTROLLER SERVICE CALLED")
   print ("Position received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 2 position set")

def handle_J3P(data):
   pub = rospy.Publisher('/SCARA_T7/joint3_position_controller/command', Float64, queue_size=1)
   print ("\nJOINT 3 POSITION CONTROLLER SERVICE CALLED")
   print ("Position received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 3 position set")

def handle_J1V(data):
   pub = rospy.Publisher('/SCARA_T7/joint1_velocity_controller/command', Float64, queue_size=1)
   print ("\nJOINT 1 VELOCITY CONTROLLER SERVICE CALLED")
   print ("Velocity received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 1 velocity set")

def handle_J2V(data):
   pub = rospy.Publisher('/SCARA_T7/joint2_velocity_controller/command', Float64, queue_size=1)
   print ("\nJOINT 2 VELOCITY CONTROLLER SERVICE CALLED")
   print ("Velocity received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 2 velocity set")

def handle_J3V(data):
   pub = rospy.Publisher('/SCARA_T7/joint3_velocity_controller/command', Float64, queue_size=1)
   print ("\nJOINT 3 VELOCITY CONTROLLER SERVICE CALLED")
   print ("Velocity received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 3 velocity set")

def SCARA_server():
   rospy.init_node('SCARA_T7_Ref_Server')
   srv_1 = rospy.Service('SCARA_T7/Joint1_Pos_Ref', PC_srv, handle_J1P)
   srv_2 = rospy.Service('SCARA_T7/Joint2_Pos_Ref', PC_srv, handle_J2P)
   srv_3 = rospy.Service('SCARA_T7/Joint3_Pos_Ref', PC_srv, handle_J3P)
   srv_4 = rospy.Service('SCARA_T7/Joint1_Vel_Ref', PC_srv, handle_J1V)
   srv_5 = rospy.Service('SCARA_T7/Joint2_Vel_Ref', PC_srv, handle_J2V)
   srv_6 = rospy.Service('SCARA_T7/Joint3_Vel_Ref', PC_srv, handle_J3V)
   print "Ready to accept Joint Reference value for SCARA."
   rospy.spin()
   
# Call function to intiate the service
if __name__ == "__main__":
     SCARA_server()
