#!/usr/bin/env python
import rospy
import sys

from unity_robotics_demo_msgs.msg import *
import copy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from std_msgs.msg import String

from json import dumps

from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import time
from copy import deepcopy
import math

from franka_gripper.msg import FGMoveGoal


class hapticviatactile:

  def __init__(self):

    #self.fg_state_pub = rospy.Publisher("/fg_internal_state", String, queue_size=1)

    #subscriber to unity
    self.ovr_inputs = rospy.Subscriber("/ovr_inputs",OVRbuttons,self.ovr_inputs_callback)
    #self.ovr_right_hand_pos_rot = rospy.Subscriber("/ovr_right_hand_pos_rot",OVRrightHandPosRot,self.ovr_right_hand_pos_rot_callback)
    #self.ovr_right_hand_vel = rospy.Subscriber("/ovr_right_hand_vel",OVRrightHandPosRot,self.ovr_right_hand_vel_callback)
    #controler buttons
    self.con_rh_one_a=None
    self.con_rh_two_b=None
    self.con_rh_asx=None
    self.con_rh_asy=None
    self.con_rh_st=None
    self.con_rh_lt=None
    self.con_lh_three_x=None
    self.con_lh_four_y=None
    self.con_lh_asx=None
    self.con_lh_asy=None
    self.con_lh_st=None
    self.con_lh_lt=None

    # #postiion
    # self.rh_pos_rot_old= None 
    # self.rh_pos_rot_acc= None
    # self.rh_twis=Twist()
    # self.rh_vel=Twist()
    # self.rh_vel_history=[]
    # self.rh_vel_window_size=5
    # self.ignore_vel=0.005
    # self.ignore_vel_angular=0.05
    # self.rh_vel_angular_factor=2
    # self.max_vel=0.1
    # self.max_vel_angular=1.

    ###tactile to haptic###
    #Puplisher to unity
    self.habptic_feedback_pub =rospy.Publisher("/haptic_feedback", HapticFeedback, queue_size=1)
    #subsciper to digit tactile sensor
    self.ready_tactile_to_haptic_1=False
    self.ready_tactile_to_haptic_2=False
    self.bridge = CvBridge()
    self.t_image_1_sub = rospy.Subscriber("/digit_cam1/image_raw",Image,self.t1callback)
    self.t_image_2_sub = rospy.Subscriber("/digit_cam2/image_raw",Image,self.t2callback)
    #tune tactile imgs
    self.ref_frame_window_size=10 
    self.num_frames_for_noise_threshold=100
    #sensor 1
    self.t_images_1=[]
    self.t_1_noise_threshold_found=False
    self.t_1_noise_threshold=None
    self.t_1_noise_threshold_counter=0
    self.t_1_noise_threshold_accum=[]
    #sensor 2
    self.t_images_2=[]
    self.t_2_noise_threshold_found=False
    self.t_2_noise_threshold=None    
    self.t_2_noise_threshold_counter=0
    self.t_2_noise_threshold_accum=[]
    #touch detection
    self.consecutive_touch_detection_frame_number=2
    self.touch_threshold_percentage=0.01
    self.t_1_curr_frames=[]
    self.p_1_touch=0
    self.t_2_curr_frames=[]
    self.p_2_touch=0
    #tactile img pup
    self.t_image_1_pub = rospy.Publisher("/tactile_image_1",Image, queue_size=1)
    self.t_image_2_pub = rospy.Publisher("/tactile_image_2",Image, queue_size=1)
    self.t_bg_image_1_pub = rospy.Publisher("/tactile_image_bg_1",Image, queue_size=1)
    self.t_bg_image_2_pub = rospy.Publisher("/tactile_image_bg_2",Image, queue_size=1)
    self.haptic_min_amp=0.05


    #puplish to the twist to the franka controler ...
    #for simulation
    #self.car_vel_pub =rospy.Publisher("/panda/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose", Twist, queue_size=1)
    #real panda
    #self.car_vel_pub =rospy.Publisher("/kth_cartesian_velocity_effort_interface_controller/panda/equilibrium_pose", Twist, queue_size=1)

    #unity cube
    #self.car_vel_pub =rospy.Publisher("/cube_vel", Twist, queue_size=1)
    


    #self.past_time=None
    #self.curr_time=None

    # #franka gripper
    # self.fg_gripper_pub = rospy.Publisher("/fg_franka_gripper/fg_franka_gripper_goal", FGMoveGoal, queue_size=1)
    # self.gripper_state_sub = rospy.Subscriber("/fg_franka_gripper/joint_states", JointState, self.grippercallback)
    # self.current_gripper_width=None
    # self.gripper_vel_close=0.005
    # self.gripper_vel_open=0.01
    # self.gripper_is_closing=False
    # self.gripper_is_opening=False
    # self.gripper_is_stopped=True

  
  # def fg_get_internal_state_json(self): 
  #   #touch detection flags digit 1

  #   dictionary = {"rh_vel_window_size": self.rh_vel_window_size, 
  #                 "ignore_vel": self.ignore_vel, 
  #                 "ignore_vel_angular": self.ignore_vel_angular, 
  #                 "rh_vel_angular_factor": self.rh_vel_angular_factor,
  #                 "max_vel": self.max_vel, 
  #                 "max_vel_angular": self.max_vel_angular, 
  #                 "ready_tactile_to_haptic_1": self.ready_tactile_to_haptic_1, 
  #                 "ready_tactile_to_haptic_2": self.ready_tactile_to_haptic_2,
  #                 "num_frames_for_noise_threshold": self.num_frames_for_noise_threshold, 
  #                 "t_1_noise_threshold_found": self.t_1_noise_threshold_found, 
  #                 "t_1_noise_threshold": self.t_1_noise_threshold, 
  #                 "t_1_noise_threshold_counter": self.t_1_noise_threshold_counter,
  #                 "t_2_noise_threshold_found": self.t_2_noise_threshold_found, 
  #                 "t_2_noise_threshold": self.t_2_noise_threshold, 
  #                 "t_2_noise_threshold_counter": self.t_2_noise_threshold_counter, 
  #                 "consecutive_touch_detection_frame_number": self.consecutive_touch_detection_frame_number,
  #                 "touch_threshold_percentage": self.touch_threshold_percentage, 
  #                 "p_1_touch": self.p_1_touch, 
  #                 "p_2_touch": self.p_2_touch, 
  #                 "haptic_min_amp": self.haptic_min_amp,
  #                 "current_gripper_width": self.current_gripper_width, 
  #                 "gripper_vel_close": self.gripper_vel_close, 
  #                 "gripper_vel_open": self.gripper_vel_open, 
  #                 "gripper_is_closing": self.gripper_is_closing,
  #                 "gripper_is_opening": self.gripper_is_opening,
  #                 "gripper_is_stopped": self.gripper_is_stopped,
                  
  #                 }

  #   state_str = dumps(dictionary)
  #   return state_str

  # def close_gripper(self):
  #   msg = FGMoveGoal()
  #   msg.width=0.000001
  #   msg.velocity=self.gripper_vel_close
  #   self.fg_gripper_pub.publish(msg)

  # def open_gripper(self):
  #   msg = FGMoveGoal()
  #   msg.width=0.07
  #   msg.velocity=self.gripper_vel_open
  #   self.fg_gripper_pub.publish(msg)


  # #stops the gripper by comanding it to the current pose
  # def fg_stop_gripper(self):
  #   msg = FGMoveGoal()
  #   msg.width=deepcopy(self.current_gripper_width)
  #   msg.velocity=0.001
  #   print("stopping gripper at : " + str(self.current_gripper_width))
  #   self.fg_gripper_pub.publish(msg)


  # def grippercallback(self, data):
  #   #print(data.position[0])
  #   self.current_gripper_width=data.position[0]*2

  def reset_vars(self):
    self.t_images_1=[]
    self.t_1_noise_threshold_found=False
    self.t_1_noise_threshold=None    
    self.t_1_noise_threshold_counter=0
    self.t_1_noise_threshold_accum=[]

    self.t_images_2=[]
    self.t_2_noise_threshold_found=False
    self.t_2_noise_threshold=None    
    self.t_2_noise_threshold_counter=0
    self.t_2_noise_threshold_accum=[]


  def auto_tune_noise_threshold(self,cv_image, digit_sensor_id):
    
    if digit_sensor_id==1:
      #fill backgound
      #build mean image with windowsize
      if len(self.t_images_1) <= self.ref_frame_window_size:
        self.t_images_1.append(cv_image)
      elif not self.t_1_noise_threshold_found:        
        #show tuning
        mean_cv_img=np.mean(self.t_images_1,axis=0).astype(np.uint8) 
        diff = self.diffrence(mean_cv_img, cv_image) 
        self.t_1_noise_threshold_accum.append(np.max(diff))
        self.t_1_noise_threshold_counter+=1
        if self.t_1_noise_threshold_counter>self.num_frames_for_noise_threshold:
          self.t_1_noise_threshold=np.mean(self.t_1_noise_threshold_accum)
          self.t_1_noise_threshold_found=True
          print("noise threshold for digit 1 set to: " + str(self.t_1_noise_threshold))    
          self.ready_tactile_to_haptic_1=False

    if digit_sensor_id==2:
      #fill backgound
      #build mean image with windowsize
      if len(self.t_images_2) <= self.ref_frame_window_size:
        self.t_images_2.append(cv_image)
      elif not self.t_2_noise_threshold_found:        
        #show tuning
        mean_cv_img=np.mean(self.t_images_2,axis=0).astype(np.uint8) 
        diff = self.diffrence(mean_cv_img, cv_image) 
        self.t_2_noise_threshold_accum.append(np.max(diff))
        self.t_2_noise_threshold_counter+=1
        if self.t_2_noise_threshold_counter>100:
          self.t_2_noise_threshold=np.mean(self.t_2_noise_threshold_accum)
          self.t_2_noise_threshold_found=True
          print("noise threshold for digit 2 set to: " + str(self.t_2_noise_threshold))
          self.ready_tactile_to_haptic_2=False
    
  def diffrence(self, base, frame):
    diff = (frame * 1.0 - base) / 255.0 + 0.5
    diff[diff < 0.5] = (diff[diff < 0.5] - 0.5) * 0.7 + 0.5
    return np.mean(np.abs(diff - 0.5), axis=-1)

  def digit_touch_detection(self, ref_frame, detection_frames, touch_threshold_percentage):
    diff_frames=[]
    for d_frame in detection_frames:
      diff=self.diffrence(ref_frame, d_frame)
      #threshold
      diff[diff<self.t_1_noise_threshold]=0
      diff[diff>=self.t_1_noise_threshold]=1
      diff_frames.append(diff)

    touch_img=diff_frames[0].copy()
    if len(diff_frames)>1:
      for diff in diff_frames[1:]:
        touch_img=touch_img.copy()*diff.copy()

    touch_img=(touch_img*255).astype(np.uint8)
    num_w_pix=np.count_nonzero(touch_img==255)
    p_w_pix=float(num_w_pix)/float((touch_img.shape[0]*touch_img.shape[1]))*100
    if p_w_pix < touch_threshold_percentage:
      p_w_pix=0
    return p_w_pix




  def t1callback(self,data):
    #do haptic feedback in the same frequencey you get frames:
    self.tactile2haptic(self.p_1_touch, self.p_2_touch)
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image=cv_image[:,100:,:]

      #keep the current frames to do the consecutive touch detection
      self.t_1_curr_frames.append(cv_image)
      if len(self.t_1_curr_frames)>self.consecutive_touch_detection_frame_number:
        self.t_1_curr_frames.pop(0)

      #tune touch threshold
      if self.ready_tactile_to_haptic_1:
        self.auto_tune_noise_threshold(cv_image, 1)
      #do the touch detection
      if self.t_1_noise_threshold_found:
        self.p_1_touch=self.digit_touch_detection(np.mean(self.t_images_1,axis=0).astype(np.uint8), self.t_1_curr_frames, self.touch_threshold_percentage)
    

    except CvBridgeError as e:
      print(e)

    #puplish bg if it's there
    if len(self.t_images_1)> self.ref_frame_window_size:
      try:
        self.t_bg_image_1_pub.publish(self.bridge.cv2_to_imgmsg(np.mean(self.t_images_1,axis=0).astype(np.uint8), "bgr8"))
      except CvBridgeError as e:
        print(e)

    try:
      self.t_image_1_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def t2callback(self,data):
    #do haptic feedback in the same frequencey you get frames:
    self.tactile2haptic(self.p_1_touch, self.p_2_touch)
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image=cv_image[:,100:,:]

      #keep the current frames to do the consecutive touch detection
      self.t_2_curr_frames.append(cv_image)
      if len(self.t_2_curr_frames)>self.consecutive_touch_detection_frame_number:
        self.t_2_curr_frames.pop(0)

      #tune touch threshold
      if self.ready_tactile_to_haptic_2:
        self.auto_tune_noise_threshold(cv_image, 2)
      #do the touch detection
      if self.t_2_noise_threshold_found:
        self.p_2_touch=self.digit_touch_detection(np.mean(self.t_images_2,axis=0).astype(np.uint8), self.t_2_curr_frames, self.touch_threshold_percentage)
    

    except CvBridgeError as e:
      print(e)

    #puplish bg if it's there
    if len(self.t_images_2)> self.ref_frame_window_size:
      try:
        self.t_bg_image_2_pub.publish(self.bridge.cv2_to_imgmsg(np.mean(self.t_images_2,axis=0).astype(np.uint8), "bgr8"))
      except CvBridgeError as e:
        print(e)

    try:
      self.t_image_2_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def tactile2haptic(self, p_t_1, p_t_2):
    amp=(p_t_1+p_t_2)/200
    amp=math.log(amp+1,100)
    #print("amp: " + str(amp))
    if amp<0.05 and amp >0.00:
      amp=self.haptic_min_amp
    #if amp <=0.001:
    #  amp=0
    self.haptic_feedback_to_unity(1, amp)
    #print("amp: " + str(amp))




  def haptic_feedback_to_unity(self, frequency, amplitude, handid="right"):
    hfmsg=HapticFeedback()
    hfmsg.frequency=frequency
    hfmsg.amplitude=amplitude
    hfmsg.handid=handid
    self.habptic_feedback_pub.publish(hfmsg)

  # def get_average_rh_vel(self,rh_vel_history):
  #   mean_rh_vel=Twist()

  #   for vel_entry in rh_vel_history:
  #     mean_rh_vel.linear.x +=vel_entry.linear.x 
  #     mean_rh_vel.linear.y +=vel_entry.linear.y 
  #     mean_rh_vel.linear.z +=vel_entry.linear.z 

  #     mean_rh_vel.angular.x +=vel_entry.angular.x
  #     mean_rh_vel.angular.y +=vel_entry.angular.y
  #     mean_rh_vel.angular.z +=vel_entry.angular.z 

  #   mean_rh_vel.linear.x /=len(rh_vel_history)
  #   mean_rh_vel.linear.y /=len(rh_vel_history)
  #   mean_rh_vel.linear.z /=len(rh_vel_history)

  #   mean_rh_vel.angular.x /=len(rh_vel_history)
  #   mean_rh_vel.angular.y /=len(rh_vel_history)
  #   mean_rh_vel.angular.z /=len(rh_vel_history)  

  #   return mean_rh_vel

  def ovr_inputs_callback(self, data):
    #update the state
    # fg_state_str = self.fg_get_internal_state_json()
    # self.fg_state_pub.publish(fg_state_str)
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



    # car_vel_twist=Twist()
    # if self.con_rh_one_a:
    #   #car_vel_twist=self.rh_twis
    #   if len(self.rh_vel_history)>0:
    #     car_vel_twist=self.get_average_rh_vel(self.rh_vel_history)

    #   else:
    #     car_vel_twist=Twist()
    # else:
    #   self.rh_vel_history=[]
    #   car_vel_twist.linear.x=0
    #   car_vel_twist.linear.y=0
    #   car_vel_twist.linear.z=0

    #   car_vel_twist.angular.x=0
    #   car_vel_twist.angular.y=0
    #   car_vel_twist.angular.z=0
    #   self.rh_pos_rot_old = None
    #   self.rh_pos_rot_acc = None


    # #print("-----")
    # #print(car_vel_twist)

    # self.car_vel_pub.publish(car_vel_twist)


    #listen for the ready button (b)
    if self.con_rh_two_b:
      print("tuning thresholds")
      self.ready_tactile_to_haptic_1=True
      self.ready_tactile_to_haptic_2=True
      self.reset_vars()


    # #monitor the shoulder trippge to coralate with gripper closing opening or stopping
    # if self.con_rh_st==1.0 and self.con_rh_lt==0:
    #   #print("button")
      
    #   if not self.gripper_is_closing:
    #     print("closing gripper")
    #     self.close_gripper()
    #     self.gripper_is_closing=True
    #     self.gripper_is_stopped=False
    # if self.con_rh_st==0 and self.con_rh_lt==1.0:
      
    #   if not self.gripper_is_opening:
    #     print("opening gripper")
    #     self.open_gripper()
    #     self.gripper_is_opening=True
    #     self.gripper_is_stopped=False
    # if self.con_rh_st==0 and self.con_rh_lt==0:
      
    #   if not self.gripper_is_stopped:
    #     print("stoping gripper")
    #     self.fg_stop_gripper()
    #     self.gripper_is_stopped=True
    #     self.gripper_is_opening=False
    #     self.gripper_is_closing=False

    # #monitor the shoulder trippge to coralate with gripper closing opening or stopping
    # if self.con_lh_st==1.0 and self.con_lh_lt==0:
    #   print("button")
      
    #   if not self.gripper_is_closing:
    #     print("closing gripper")
    #     self.close_gripper()
    #     self.gripper_is_closing=True
    #     self.gripper_is_stopped=False
    # if self.con_lh_st==0 and self.con_lh_lt==1.0:
      
    #   if not self.gripper_is_opening:
    #     print("opening gripper")
    #     self.open_gripper()
    #     self.gripper_is_opening=True
    #     self.gripper_is_stopped=False
    # if self.con_lh_st==0 and self.con_lh_lt==0:
      
    #   if not self.gripper_is_stopped:
    #     print("stoping gripper")
    #     self.fg_stop_gripper()
    #     self.gripper_is_stopped=True
    #     self.gripper_is_opening=False
    #     self.gripper_is_closing=False




  # def ovr_right_hand_vel_callback(self, data):
  #   #print("callbckus")
  #   #print(data)

  #   # self.rh_vel.linear.x=data.pos_x    
  #   # self.rh_vel.linear.y=data.pos_y
  #   # self.rh_vel.linear.z=data.pos_z
  #   # self.rh_vel.angular.x=data.eul_x
  #   # self.rh_vel.angular.y=data.eul_y
  #   # self.rh_vel.angular.z=data.eul_z



  #   # self.rh_vel.linear.x=-data.pos_z    
  #   # self.rh_vel.linear.y=data.pos_x
  #   # self.rh_vel.linear.z=data.pos_y
  #   # self.rh_vel.angular.x=data.eul_y
  #   # self.rh_vel.angular.y=data.eul_x
  #   # self.rh_vel.angular.z=-data.eul_z


  #   # self.rh_vel.linear.x=-data.pos_x    
  #   # self.rh_vel.linear.y=-data.pos_z
  #   # self.rh_vel.linear.z=data.pos_y
  #   # self.rh_vel.angular.x=data.eul_y
  #   # self.rh_vel.angular.y=-data.eul_z
  #   # self.rh_vel.angular.z=-data.eul_x



  #   # self.rh_vel.linear.x=-data.pos_z   
  #   # self.rh_vel.linear.y=data.pos_x
  #   # self.rh_vel.linear.z=data.pos_y
  #   # self.rh_vel.angular.x=data.eul_y
  #   # self.rh_vel.angular.y=data.eul_x
  #   # self.rh_vel.angular.z=-data.eul_z


    
  #   self.rh_vel.linear.x=-data.pos_x   
  #   self.rh_vel.linear.y=-data.pos_z
  #   self.rh_vel.linear.z=data.pos_y
  #   self.rh_vel.angular.x=data.eul_y
  #   self.rh_vel.angular.y=-data.eul_z
  #   self.rh_vel.angular.z=-data.eul_x



  #   # #hard assignments of axis?
  #   # self.rh_vel.linear.x=-data.pos_x 
  #   # self.rh_vel.linear.y=-data.pos_z 
  #   # self.rh_vel.linear.z=data.pos_y 

  #   # self.rh_vel.angular.x=data.eul_y #/self.rh_vel_angular_factor 
  #   # self.rh_vel.angular.y= -data.eul_z #/self.rh_vel_angular_factor 
  #   # self.rh_vel.angular.z=-data.eul_x #/self.rh_vel_angular_factor 


  #   # # #hard assignments of axis?
  #   # self.rh_vel.linear.x=data.pos_x
  #   # self.rh_vel.linear.y=data.pos_z
  #   # self.rh_vel.linear.z=data.pos_y

  #   # self.rh_vel.angular.x=data.eul_y 
  #   # self.rh_vel.angular.y= data.eul_z
  #   # self.rh_vel.angular.z=data.eul_x


  #   # # #hard assignments of axis?
  #   # self.rh_vel.linear.x=-data.pos_x
  #   # self.rh_vel.linear.y=data.pos_z*0
  #   # self.rh_vel.linear.z=data.pos_y*0

  #   # self.rh_vel.angular.x=data.eul_y *0
  #   # self.rh_vel.angular.y= data.eul_z*0
  #   # self.rh_vel.angular.z=data.eul_x*0



  #   if self.rh_vel.linear.x > -self.ignore_vel and self.rh_vel.linear.x < self.ignore_vel:
  #     self.rh_vel.linear.x =0
  #   if self.rh_vel.linear.y > -self.ignore_vel and self.rh_vel.linear.y < self.ignore_vel:
  #     self.rh_vel.linear.y =0
  #   if self.rh_vel.linear.z > -self.ignore_vel and self.rh_vel.linear.z < self.ignore_vel:
  #     self.rh_vel.linear.z =0
  #   if self.rh_vel.angular.x > -self.ignore_vel_angular and self.rh_vel.angular.x < self.ignore_vel_angular:
  #     self.rh_vel.angular.x =0
  #   if self.rh_vel.angular.y > -self.ignore_vel_angular and self.rh_vel.angular.y < self.ignore_vel_angular:
  #     self.rh_vel.angular.y =0
  #   if self.rh_vel.angular.z > -self.ignore_vel_angular and self.rh_vel.angular.z < self.ignore_vel_angular:
  #     self.rh_vel.angular.z =0

  #   self.rh_vel.linear.x = np.clip(self.rh_vel.linear.x, - self.max_vel, self.max_vel)
  #   self.rh_vel.linear.y = np.clip(self.rh_vel.linear.y, - self.max_vel, self.max_vel)
  #   self.rh_vel.linear.z = np.clip(self.rh_vel.linear.z, - self.max_vel, self.max_vel)

  #   self.rh_vel.angular.x = np.clip(self.rh_vel.angular.x, - self.max_vel_angular, self.max_vel_angular)
  #   self.rh_vel.angular.y = np.clip(self.rh_vel.angular.y, - self.max_vel_angular, self.max_vel_angular)
  #   self.rh_vel.angular.z = np.clip(self.rh_vel.angular.z, - self.max_vel_angular, self.max_vel_angular)

  #   self.rh_vel_history.append(self.rh_vel)
  #   #print("....")
  #   #print(self.rh_vel)

  #   if len(self.rh_vel_history)> self.rh_vel_window_size:
  #     self.rh_vel_history.pop(0)


    




  # def ovr_right_hand_pos_rot_callback(self, data):
  #   if self.rh_pos_rot_old is None:
  #     self.rh_pos_rot_old=OVRrightHandPosRot()
  #     self.rh_pos_rot_old = data
  #     self.past_time = time.time()
  #     #print(self.rh_pos_rot_old)
  #   elif self.rh_pos_rot_acc is None:
  #     self.rh_pos_rot_acc=OVRrightHandPosRot()
  #     self.rh_pos_rot_acc=data
  #     self.curr_time = time.time()
  #   else:
  #     self.rh_pos_rot_old= copy.copy(self.rh_pos_rot_acc)
  #     self.rh_pos_rot_acc=data
  #     #here we also do the axis assignment!!!!
  #     rh_twis=Twist()
  #     dt=self.curr_time-self.past_time
  #     rh_twis.linear.x=(self.rh_pos_rot_old.pos_x -self.rh_pos_rot_acc.pos_x)/dt
  #     #y in unity corrseesbonds to z in real world!
  #     rh_twis.linear.y=(self.rh_pos_rot_old.pos_z -self.rh_pos_rot_acc.pos_z)/dt
  #     rh_twis.linear.z=(self.rh_pos_rot_old.pos_y -self.rh_pos_rot_acc.pos_y)/dt
  #     #rot in euler (wraping around is not considerd!!!)
  #     rh_twis.angular.x=0#(self.rh_pos_rot_old.eul_x -self.rh_pos_rot_acc.eul_x)/dt
  #     rh_twis.angular.y=0#(self.rh_pos_rot_old.eul_y -self.rh_pos_rot_acc.eul_y)/dt
  #     rh_twis.angular.z=0#(self.rh_pos_rot_old.eul_z -self.rh_pos_rot_acc.eul_z)/dt
  #     self.past_time=copy.copy(self.curr_time)
  #     self.curr_time=time.time()
  #     self.rh_twis = self.sanatise_twist(rh_twis)


  #   #print(self.rh_twis)


  # def sanatise_twist(self, twist,v_min_lin=0.0001, v_max_lin=0.5, v_maxignore_lin=10,v_min_ang=0, v_max_ang=0.01, v_maxignore_ang=50 ):
  #   twist.linear.x = self.sanatise_twist_indu(twist.linear.x,v_min_lin, v_max_lin, v_maxignore_lin)
  #   twist.linear.y = self.sanatise_twist_indu(twist.linear.y,v_min_lin, v_max_lin, v_maxignore_lin)
  #   twist.linear.z = self.sanatise_twist_indu(twist.linear.z,v_min_lin, v_max_lin, v_maxignore_lin)
  #   twist.angular.x = self.sanatise_twist_indu(twist.angular.x,v_min_ang, v_max_ang, v_maxignore_ang)
  #   twist.angular.y = self.sanatise_twist_indu(twist.angular.y,v_min_ang, v_max_ang, v_maxignore_ang)
  #   twist.angular.z = self.sanatise_twist_indu(twist.angular.z,v_min_ang, v_max_ang, v_maxignore_ang)

  #   return twist

  # def sanatise_twist_indu(self, twist_indu, v_min, v_max, v_maxignore):
  #   #print("---")
  #   #print(twist_indu)
  #   #print(v_maxignore)
  #   sign=None
  #   if twist_indu<0:
  #     sign=-1
  #   else:
  #     sign=1

  #   twist_indu=np.abs(twist_indu)
  #   if twist_indu > v_maxignore:
  #     return 0
  #   elif twist_indu < v_min:
  #     return 0
  #   else:
  #     return sign*np.clip(twist_indu, 0, v_max)





# def print_commands():
#   print("-------------------------")
#   print("tt - tune thresholds")
#   print("ttd - test touch detection")
#   print("up1 - unplugg only digit 1")
#   print("up2 - unplugg only digit 2")
#   print("upb - unplugg both digits")
#   print("upbns -unplug basline no slip")
#   print("mu -move slowly up")
#   print("sm - stop moving")
#   print("com - plot commands")
#   print("dg - debug ask for gripper")
#   print("dsc - debug swap controler (joint to car")
#   print("dpg - depub go to pregrasp pose")
#   print("dpg2 - depub go to grasp pose")
#   print("dscv -debug swap (car to vel")
#   print("-------------------------")


def main(args):
  rospy.init_node('hapticviatactile', anonymous=True)


  hvt = hapticviatactile()  

  r = rospy.Rate(1000) #1000hz
  #print_commands()
  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)