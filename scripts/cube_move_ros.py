#!/usr/bin/env python
import rospy
import sys

from quest2_ros_msg.msg import *
from geometry_msgs.msg import Twist 
import numpy as np 



class rosunitycubecom:

  def __init__(self):

    #subscriber to unity
    self.ovr_inputs = rospy.Subscriber("/ovr_inputs",OVRbuttons,self.ovr_inputs_callback)
    self.ovr_right_hand_vel = rospy.Subscriber("/ovr_right_hand_vel",OVRHandPosRotVel,self.ovr_right_hand_vel_callback)    
    self.ovr_left_hand_vel = rospy.Subscriber("/ovr_left_hand_vel",OVRHandPosRotVel,self.ovr_left_hand_vel_callback)
    
    #Puplisher to unity
    self.cube_vel_pub =rospy.Publisher("/cube_vel", Twist, queue_size=1)

   

  def ovr_inputs_callback(self, data):
    #read the buttons
    self.con_rh_one_a=data.rh_one_a
    self.con_rh_two_b=data.rh_two_b
    self.con_rh_asx=data.rh_asx
    self.con_rh_asy=data.rh_asy
    self.con_rh_st=data.rh_st
    self.con_rh_lt=data.rh_lt
    self.con_lh_three_x=data.lh_three_x
    self.con_lh_four_y=data.lh_four_y
    self.con_lh_asx=data.lh_asx
    self.con_lh_asy=data.lh_asy
    self.con_lh_st=data.lh_st
    self.con_lh_lt=data.lh_lt


  def ovr_right_hand_vel_callback(self, data):
    ovr_right_hand_vel =Twist()
    ovr_right_hand_vel.linear.x=data.vel_x    
    ovr_right_hand_vel.linear.y=data.vel_y
    ovr_right_hand_vel.linear.z=data.vel_z
    ovr_right_hand_vel.angular.x=data.vel_eul_x
    ovr_right_hand_vel.angular.y=data.vel_eul_y
    ovr_right_hand_vel.angular.z=data.vel_eul_z

    #if A is pressed
    if self.con_rh_one_a:
      print("puplishing right cube vel")
      print(ovr_right_hand_vel)
      self.cube_vel_pub.publish(ovr_right_hand_vel)

  def ovr_left_hand_vel_callback(self, data):
    ovr_left_hand_vel =Twist()
    ovr_left_hand_vel.linear.x=data.vel_x    
    ovr_left_hand_vel.linear.y=data.vel_y
    ovr_left_hand_vel.linear.z=data.vel_z
    ovr_left_hand_vel.angular.x=data.vel_eul_x
    ovr_left_hand_vel.angular.y=data.vel_eul_y
    ovr_left_hand_vel.angular.z=data.vel_eul_z

    #if X is pressed
    if self.con_lh_three_x:
      print("puplishing left cube vel")
      print(ovr_left_hand_vel)
      self.cube_vel_pub.publish(ovr_left_hand_vel)




def main(args):
  print("starting cube teleop")
  rospy.init_node('telecube_ros', anonymous=True)


  telecube = rosunitycubecom()  

  r = rospy.Rate(1000) #1000hz
  #print_commands()
  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)