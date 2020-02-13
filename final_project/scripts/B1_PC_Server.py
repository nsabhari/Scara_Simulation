#!/usr/bin/env python 

'''
RBE 500 Final Project - Part B1 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 30 - Nov - 2019 

# Server(Node) Name : SCARA_T7_J3_Server
# Service Name : SCARA_T7/Joint3_Ref (Service Type Name : PC_srv)
'''

from final_project.srv import *
import rospy
from std_msgs.msg import Float64
import numpy as np
np.set_printoptions(suppress=True)

def handle_SCARA_PC(data):
   pub = rospy.Publisher('/SCARA_T7/joint3_position_controller/command', Float64, queue_size=1)
   print ("\nJOINT 3 POSITION CONTROLLER SERVICE CALLED")
   print ("Position received : " + str(data.input))
   pub.publish(data.input)
   return PC_srvResponse("Joint 3 position set")

def SCARA_server():
   rospy.init_node('SCARA_T7_J3_Server')
   srv = rospy.Service('SCARA_T7/Joint3_Ref', PC_srv, handle_SCARA_PC)
   print "Ready to accept Joint 3 Reference value for SCARA."
   rospy.spin()
   
# Call function to intiate the service
if __name__ == "__main__":
     SCARA_server()
