#!/usr/bin/env python 

'''
RBE 500 Final Project - Part B2 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 30 - Nov - 2019 

# Node Name : SCARA_T7_J3_SetRef
# Service Called : SCARA_T7/Joint3_Ref
# Topics Subscribed : /SCARA_T7/joint_states, /SCARA_T7/joint3_position_controller/state
'''

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
from final_project.srv import *
import time
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)
flag_rec = 0
x_t1 = []
x_t2 = []
y_Ref = []
y_Meas = []
ctr = 0

def callback_JointMeas(data):
   global flag_rec,x_t2,y_Meas
   if flag_rec == 1:
      t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
      pos = data.position[0]*100
      x_t2.append(t)
      y_Meas.append(pos)
      print ("Time (sec) : " + str(t) + ", Position (cms) :" + str(pos))

def callback_JointRef(data):
   global flag_rec,x_t1,y_Ref
   if flag_rec == 1:
      t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
      pos = data.set_point*100
      x_t1.append(t)
      y_Ref.append(pos)
   
# Call function to intiate the service
if __name__ == "__main__":
   rospy.init_node('SCARA_T7_J3_SetRef', anonymous = False)
   print("\nWaiting for the required services...")
   rospy.wait_for_service('SCARA_T7/Joint3_Ref')
   J3_service = rospy.ServiceProxy('SCARA_T7/Joint3_Ref', PC_srv)
   print("Required services up and running...")
   print("Starting subscriber")
   rospy.Subscriber("/SCARA_T7/joint_states", JointState, callback_JointMeas)
   rospy.Subscriber('/SCARA_T7/joint3_position_controller/state', JointControllerState, callback_JointRef)
   print("Joint 3 Limits : [-3 , 3] cms")
   flag = 0
   while (flag == 0):
      x = raw_input("Enter the J3 Reference Value (Type \"q\" to exit) : ")
      if x == "q":
         flag = 1
      else:
         q3 = float(x)/100
         ctr += 1
         # Resetting global variables
         x_t1 = []
         x_t2 = []
         y_Ref = []
         y_Meas = []
         # Start recording data here (Record data for 1 second)
         flag_rec = 1
         time.sleep(0.5)
         out = J3_service(q3)
         time.sleep(0.5)
         flag_rec = 0
         # Plotting the graph (Graphs get saved in the home folder)
         plt.clf()
         plt.plot(x_t2, y_Meas,'g',x_t1, y_Ref,'--r')
         plt.ylim(-4, 5)
         plt.xlabel("Time in secs")
         plt.ylabel("Position in cms")
         plt.title("Joint 3 Reference vs Measured")
         plt.legend(["Joint 3 Measured","Joint 3 Reference"])
         plt.savefig('Graph_'+str(ctr)+'.png')
