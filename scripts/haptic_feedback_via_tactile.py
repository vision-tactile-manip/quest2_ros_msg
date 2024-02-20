#!/usr/bin/env python
import rospy
import sys

from quest2_ros_msg.msg import *
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



class hapticviatactile:

  def __init__(self):


    #subscriber to unity
    self.ovr_inputs = rospy.Subscriber("/ovr_inputs",OVRbuttons,self.ovr_inputs_callback)
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
    self.haptic_feedback_to_unity(1, amp)




  def haptic_feedback_to_unity(self, frequency, amplitude, handid="right"):
    hfmsg=HapticFeedback()
    hfmsg.frequency=frequency
    hfmsg.amplitude=amplitude
    hfmsg.handid=handid
    self.habptic_feedback_pub.publish(hfmsg)


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



    #listen for the ready button (b)
    if self.con_rh_two_b:
      print("tuning thresholds")
      self.ready_tactile_to_haptic_1=True
      self.ready_tactile_to_haptic_2=True
      self.reset_vars()



def main(args):
  rospy.init_node('hapticviatactile', anonymous=True)


  hvt = hapticviatactile()  

  r = rospy.Rate(1000) #1000hz
  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)