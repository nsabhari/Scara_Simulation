#!/usr/bin/env python 

'''
RBE 500 Final Project - Part C2 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 13 - Dec - 2019 

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
lim = [0,0]
ctr = 0

mode = 5  # (1-3) Position, (4-6) Velocity

def callback_JointMeas(data):
   global flag_rec,x_t2,y_Meas,mode
   if flag_rec == 1:
      t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
      if mode == 1 or mode == 2:
         Meas = np.degrees(data.position[0])
      elif mode == 3 :
         Meas = data.position[0]*100
      elif mode == 4 or mode == 5:
         Meas = np.degrees(data.velocity[1])
      elif mode == 6:
         Meas = data.velocity[0]*100
      x_t2.append(t)
      y_Meas.append(Meas)
      if (mode <= 3):
         print ("Time (sec) : " + str(t) + ", Position :" + str(Meas))
      else:
         print ("Time (sec) : " + str(t) + ", Velocity :" + str(Meas))

def callback_JointRef(data):
   global flag_rec,x_t1,y_Ref,mode
   if flag_rec == 1:
      t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
      if mode == 3 or mode == 6 :
         Ref = data.data.set_point*100
      else:
         Ref = np.degrees(data.set_point)
      x_t1.append(t)
      y_Ref.append(Ref)
   
# Call function to intiate the service
if __name__ == "__main__":
   rospy.init_node('SCARA_T7_SetRef', anonymous = False)
   if (mode <= 3):
      srv_name = 'SCARA_T7/Joint' + str(mode) + '_Pos_Ref'
      sub_name = '/SCARA_T7/joint' + str(mode) + '_position_controller/state'
      title = 'Joint ' + str(mode) + ' Reference vs Measured'
      legend = ['Joint ' + str(mode) + ' Measured','Joint ' + str(mode) + ' Reference']
      if mode == 3:
         y_label = 'Position in cms'
         lim[0] = -3
         lim[1] = 3
      else:
         y_label = 'Position in deg'
         if mode == 1:
            lim[0] = -90
            lim[1] = 90
         else:
            lim[0] = -135
            lim[1] = 135
   else:
      srv_name = 'SCARA_T7/Joint' + str(mode-3) + '_Vel_Ref'
      sub_name = '/SCARA_T7/joint' + str(mode-3) + '_velocity_controller/state'
      title = "Joint " + str(mode-3) + ' Reference vs Measured'
      legend = ['Joint ' + str(mode-3) + ' Measured','Joint ' + str(mode-3) + ' Reference']
      if mode == 6:
         y_label = 'Velocity in cms/sec'
         lim[0] = -500
         lim[1] = 500
      else:
         y_label = 'Velocity in deg/sec'
         if mode == 4:
            lim[0] = -456
            lim[1] = 456
         else:
            lim[0] = -400 #-688
            lim[1] = 400 #688
 
   print("\nWaiting for the required services...")
   rospy.wait_for_service(srv_name)
   SR_service = rospy.ServiceProxy(srv_name, PC_srv)
   print("Required services up and running...")
   print("Starting subscriber")
   rospy.Subscriber("/SCARA_T7/joint_states", JointState, callback_JointMeas)
   rospy.Subscriber(sub_name, JointControllerState, callback_JointRef)
   flag = 0
   while (flag == 0):
      x = raw_input("Enter the Reference Value (Type \"q\" to exit) : ")
      if x == "q":
         flag = 1
      else:
         if mode == 3 or mode == 6:
            val = float(x)/100
         else:
            val = np.radians(float(x))
         ctr += 1
         # Resetting global variables
         x_t1 = []
         x_t2 = []
         y_Ref = []
         y_Meas = []
         # Start recording data here (Record data for 1 second)
         flag_rec = 1
         time.sleep(0.25)
         out = SR_service(val)
         time.sleep(0.75)
         flag_rec = 0
         # Plotting the graph (Graphs get saved in the home folder)
         plt.clf()
         plt.plot(x_t2, y_Meas,'g',x_t1, y_Ref,'--r')
         
         plt.ylim(lim[0]*1.2, lim[1]*1.5)
         plt.xlabel("Time in secs")
         plt.ylabel(y_label)
         plt.title(title)
         plt.legend(legend)
         plt.grid(True)
         plt.show()
         #plt.savefig('Graph_'+str(mode)+'_'+str(ctr)+'.png')
