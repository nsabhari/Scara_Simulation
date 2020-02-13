#!/usr/bin/env python 

'''
RBE 500 Final Project - Part C1 : Team 7
Team Members : Neel Dhanaraj, Sabhari Natarajan, Sidharth Sharma
Last Modified : 13 - Dec - 2019 

# Nodw Name : SCARA_T7_Switch_Controller
# Service Called : /SCARA_T7/controller_manager/switch_controller
                   (Service Type Name : SwitchController)
'''
import rospy
import time
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from final_project.srv import *
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

q_read = [0,0,0]
q_dot_ref = [0,0,0]
q_dot_read = [0,0,0]

flag_rec = 0
q1_meas = []
q2_meas = []
q1_ref = []
q2_ref = []
x_Pos = []
y_Pos = []
x_t = []
#x_t2 = []

def callback_JointState(data):
   global q_read,flag_rec,q1_ref,q2_ref,q1_meas,q2_meas
   t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
   q_read[0] = np.degrees(data.position[0])
   q_read[1] = np.degrees(data.position[1])
   q_read[2] = 0
   q_dot_read[0] = np.degrees(data.velocity[0])
   q_dot_read[1] = np.degrees(data.velocity[1])
   q_dot_read[2] = 0
   if flag_rec == 1:
      x_t.append(t)
      q1_ref.append(np.degrees(q_dot_ref[0]))
      q2_ref.append(np.degrees(q_dot_ref[1]))
      q1_meas.append(q_dot_read[0])
      q2_meas.append(q_dot_read[1])
      pose_calc = FK_service([q_read[0],q_read[1],0])
      #print(data.position,pose_calc)
      x_Pos.append(pose_calc.pose[0])
      y_Pos.append(pose_calc.pose[1])
      
if __name__ == "__main__":
   rospy.init_node('SCARA_T7_Switch_Controller', anonymous = False)
   print("\nWaiting for the required services...")
   rospy.wait_for_service('/SCARA_T7/controller_manager/switch_controller')
   rospy.wait_for_service('SCARA_T7_IVK')
   rospy.wait_for_service('SCARA_T7_FK')
   SC_service = rospy.ServiceProxy('/SCARA_T7/controller_manager/switch_controller', SwitchController)
   IVK_service = rospy.ServiceProxy('SCARA_T7_IVK', VK_srv)
   FK_service = rospy.ServiceProxy('SCARA_T7_FK', FK_srv)
   print("Required services up and running...")

   rospy.Subscriber("/SCARA_T7/joint_states", JointState, callback_JointState)
   pub_q1_P = rospy.Publisher('/SCARA_T7/joint1_position_controller/command', Float64, queue_size=1)
   pub_q2_P = rospy.Publisher('/SCARA_T7/joint2_position_controller/command', Float64, queue_size=1)
   pub_q1_V = rospy.Publisher('/SCARA_T7/joint1_velocity_controller/command', Float64, queue_size=1)
   pub_q2_V = rospy.Publisher('/SCARA_T7/joint2_velocity_controller/command', Float64, queue_size=1)
   res = SC_service(['joint1_position_controller','joint2_position_controller'], 
                       ['joint1_velocity_controller','joint2_velocity_controller'], 2)
   time.sleep(1)

   q1_meas = []
   q2_meas = []
   q1_ref = []
   q2_ref = []

   q1 = -90
   q2 = 90
   #q1 = raw_input("Enter the initial q1 : ")
   #q2 = raw_input("Enter the initial q2 : ")
   pub_q1_P.publish(np.radians(float(q1)))
   pub_q2_P.publish(np.radians(float(q2)))
   time.sleep(1)
   print("Position Set , Switching to Velocity Control")
   res = SC_service(['joint1_velocity_controller','joint2_velocity_controller'], 
                    ['joint1_position_controller','joint2_position_controller'], 2)              
   if (res.ok == True):
      print("Success\n")

   pub_q1_V.publish(0)
   pub_q2_V.publish(0)
   
   #vx = raw_input("Enter the velocity in x (m/s) : ")
   #vy = raw_input("Enter the velocity in y (m/s) : ")
   #vz = 0
   #dur = raw_input("Enter duration (sec) : ")

   vx = 0
   vy = 0.2
   vz = 0
   dur = 2
   
   flag_rec = 1
   time.sleep(0.25)
   cur_time = time.time()
   end_time = cur_time+float(dur)
   while(1):
      q_dot = IVK_service([q_read[0],q_read[1],q_read[2],float(vx),float(vy),vz])
      q_dot_ref[0] = q_dot.output[0]
      q_dot_ref[1] = q_dot.output[1]
      
      if (cur_time>end_time):
         q_dot_ref = [0,0,0]
         break
      pub_q1_V.publish(q_dot.output[0])
      pub_q2_V.publish(q_dot.output[1])
      cur_time = time.time()

   pub_q1_V.publish(0)
   pub_q2_V.publish(0)
   
   time.sleep(0.1)

   vx = 0.2
   vy = 0
   vz = 0
   dur = 0.5

   cur_time = time.time()
   end_time = cur_time+float(dur)
   while(1):
      q_dot = IVK_service([q_read[0],q_read[1],q_read[2],float(vx),float(vy),vz])
      q_dot_ref[0] = q_dot.output[0]
      q_dot_ref[1] = q_dot.output[1]
      
      if (cur_time>end_time):
         q_dot_ref = [0,0,0]
         break
      pub_q1_V.publish(q_dot.output[0])
      pub_q2_V.publish(q_dot.output[1])
      cur_time = time.time()

   pub_q1_V.publish(0)
   pub_q2_V.publish(0)
   
   time.sleep(0.1)

   vx = 0
   vy = -0.2
   vz = 0
   dur = 2

   cur_time = time.time()
   end_time = cur_time+float(dur)
   while(1):
      q_dot = IVK_service([q_read[0],q_read[1],q_read[2],float(vx),float(vy),vz])
      q_dot_ref[0] = q_dot.output[0]
      q_dot_ref[1] = q_dot.output[1]
      
      if (cur_time>end_time):
         q_dot_ref = [0,0,0]
         break
      pub_q1_V.publish(q_dot.output[0])
      pub_q2_V.publish(q_dot.output[1])
      cur_time = time.time()

   pub_q1_V.publish(0)
   pub_q2_V.publish(0)
   
   time.sleep(0.1)

   vx = -0.2
   vy = 0
   vz = 0
   dur = 0.5

   cur_time = time.time()
   end_time = cur_time+float(dur)
   while(1):
      q_dot = IVK_service([q_read[0],q_read[1],q_read[2],float(vx),float(vy),vz])
      q_dot_ref[0] = q_dot.output[0]
      q_dot_ref[1] = q_dot.output[1]
      
      if (cur_time>end_time):
         q_dot_ref = [0,0,0]
         break
      pub_q1_V.publish(q_dot.output[0])
      pub_q2_V.publish(q_dot.output[1])
      cur_time = time.time()

   pub_q1_V.publish(0)
   pub_q2_V.publish(0)
   
   time.sleep(0.1)

   flag_rec = 0

   res = SC_service(['joint1_position_controller','joint2_position_controller'], 
                    ['joint1_velocity_controller','joint2_velocity_controller'], 2)



   plt.clf()
      
   plt.subplot(221).set_title("Joint 1 Velocity")
   plt.subplot(221), plt.plot(x_t, q1_meas,'g',x_t, q1_ref,'--r')
   plt.subplot(221), plt.ylim(-400, 400)
   plt.subplot(221), plt.grid(True)
   plt.subplot(222).set_title("Joint 2 Velocity")
   plt.subplot(222), plt.plot(x_t, q2_meas,'g',x_t, q2_ref,'--r')
   plt.subplot(222), plt.ylim(-700, 700)
   plt.subplot(222), plt.grid(True)
   plt.subplot(223).set_title("End Effector X Position")
   plt.subplot(223), plt.plot(x_t, x_Pos,'g')
   plt.subplot(223), plt.ylim(-0.5, 0.5)
   plt.subplot(223), plt.grid(True)
   plt.subplot(224).set_title("End Effector Y Position")
   plt.subplot(224), plt.plot(x_t, y_Pos,'g',)
   plt.subplot(224), plt.ylim(-0.5, 0.5)
   plt.subplot(224), plt.grid(True)
   plt.show()
      

   
