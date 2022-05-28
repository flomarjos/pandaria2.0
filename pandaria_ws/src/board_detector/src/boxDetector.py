#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from helper_functions.utils import computeDepth2Coordinates, preprocessImage, computeMeanOrange, computeYawRotation, refinedOrientation
from scipy.spatial import procrustes
import tf2_ros
import tf_conversions
import math

class board_detector:
   
  
  def __init__(self):
     self.tfBuffer_ = tf2_ros.Buffer()
     listener = tf2_ros.TransformListener(self.tfBuffer_)
     image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
     depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
     info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, \
                                                                  self.cameraInfo_callback)
     self.posePub = rospy.Publisher("/detector/pose", Pose, queue_size=10)
     ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
     
     ts.registerCallback(self.image_callback)
   
  def cameraInfo_callback(self, info):
     self.camera_intrinsics = info
    
  def image_callback(self, image, depth_Image):
    """
        Callback function that finds the board and publishes the center 3D point 
    
    Args:
        img: RGB data in ROS Image msg format
        depth_img: depth map in ROS Image msg format
    """
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    cv_depth_img = bridge.imgmsg_to_cv2(depth_Image, 'passthrough')


    # create copy and preprocess image with canny function
    cv_img_cont = cv_img.copy()
    cv_img_proc = preprocessImage(cv_img)

    # find and iterate through contours to find shapes in defined area interval
    _, contours, _ = cv2.findContours(cv_img_proc,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cX = 0
    cY = 0   
    ref_point = [0,0]
    ref_point2 = [0,0]
    euler_z = 0
    orient_point = [0,0]
    rectangle = np.array([[1, 4], [6,4], [6, 1], [1, 1]], 'd')
    #print("rectangle: ", rectangle)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.1*peri, True)
        objcorners = len(approx)
        rospy.loginfo("Peri: %f", peri)
        
        #                     750<peri<950
                            #1000<peri<1200
        if objcorners==4 and 700<peri:#<2500: 
            a = np.squeeze(approx)
            #print("rectangle: ", rectangle)
            #print("a: ",a )
            [_,_,disparity] = procrustes(rectangle, a)
            if disparity>0.4:
                print("disparity: ", disparity)                
                continue
            #print(area)
            #print("approx", approx)
            #print("objcorners: ", objcorners)
            #print("peri: ", peri)
            #print("area: ", area)
            cv2.drawContours(cv_img_cont, cnt, -1, (0,200,255), 2)
            # compute and drawcenter point
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(cv_img_cont, (cX, cY), 2, (0,0,255), -1)
            old_dist = 999999999999999
            ref_idx = 9999     
            for i,corner in enumerate(approx):
              #print("index: ", i)
              #print("corner: ",corner)
              old_orient_point = computeMeanOrange(cv_img)
              if np.any(old_orient_point) != None:
                cv2.circle(cv_img_cont, (int(old_orient_point[0]), int(old_orient_point[1])), 10, (0,255,0), -1)
                #print("orient_point", orient_point)
                dist = np.linalg.norm(corner-old_orient_point)
                #print("dist_vec" , dist)
                if dist<old_dist:
                  old_dist = dist.copy()
                  ref_idx = i
                  ref_point = np.squeeze(corner)
                  orient_point = old_orient_point.copy()
            if orient_point[0]!=0 and orient_point[1]!=0:
               euler_z, ref_point2 = computeYawRotation(ref_idx, approx)
               cv2.circle(cv_img_cont, (ref_point[0], ref_point[1]), 10, (255,0,0), -1)
               line_thickness = 2
               cv2.line(cv_img_cont, (ref_point[0], ref_point[1]), (ref_point2[0], ref_point2[1]), (0, 0, 255), thickness=line_thickness)
              #print(euler_z)
            #print("ref_point: ", ref_point)
            #print("ref_point2: ",ref_point2)
                            
    #obtain depth value from depth image 
    if cX > 0 and orient_point[0]>0:
        #print("dentro_ref_point: ", ref_point)
        #print("dentro_ref_point2: ",ref_point2)
        depth_cen = cv_depth_img[cY, cX]
        depth_refPoint = cv_depth_img[cY, cX]#cv_depth_img[ref_point[1], ref_point[0]]
        
        # try:
        #   trans = self.tfBuffer_.lookup_transform('panda_link0', 'panda_link8', rospy.Time(0))
        #   depth_cen = int((trans.transform.translation.z - 0.16445)*1000)
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #   depth_cen = 150
        
        xc, yc, zc = computeDepth2Coordinates(depth_cen, cX, cY, self.camera_intrinsics)
        ref_translation = computeDepth2Coordinates(depth_refPoint, ref_point[0], ref_point[1], self.camera_intrinsics)
        # initialize pose
        center_pose = Pose()
        center_pose.position.x = xc
        center_pose.position.y = yc
        center_pose.position.z = zc
        center_pose.orientation.x = 0
        center_pose.orientation.y = 0
        center_pose.orientation.z = 0
        center_pose.orientation.w = 1

        # Create transform
        t = TransformStamped()
        t_cen = TransformStamped()
        t_ref = TransformStamped()####

        tfBroadcaster = tf2_ros.TransformBroadcaster()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera"
        t.child_frame_id = "taskboard_reference"
        t.transform.translation.x = ref_translation[0]
        t.transform.translation.y = ref_translation[1]
        t.transform.translation.z = ref_translation[2]
        #print("-------------------taskboard_reference z translation:", ref_translation[2])

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, euler_z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t_cen.header.stamp = rospy.Time.now()
        t_cen.header.frame_id = "camera"
        t_cen.child_frame_id = "taskboard_center"
        t_cen.transform.translation.x = xc
        t_cen.transform.translation.y = yc
        t_cen.transform.translation.z = zc
        #print("-------------------taskboard_center z translation:", zc)
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, euler_z)
        t_cen.transform.rotation.x = q[0]
        t_cen.transform.rotation.y = q[1]
        t_cen.transform.rotation.z = q[2]
        t_cen.transform.rotation.w = q[3] 

        ####
        euler_z_refined = refinedOrientation(cv_img_proc)
        print("_euler_z_refined: ", euler_z_refined)
        if euler_z_refined is not None and not math.isnan(euler_z_refined):
          t_ref.header.stamp = rospy.Time.now()
          t_ref.header.frame_id = "camera"
          t_ref.child_frame_id = "taskboard_center_ref"
          t_ref.transform.translation.x = xc
          t_ref.transform.translation.y = yc
          t_ref.transform.translation.z = zc
          #print("-------------------taskboard_center z translation:", zc)
          q = tf_conversions.transformations.quaternion_from_euler(0, 0, euler_z_refined)
          t_ref.transform.rotation.x = q[0]
          t_ref.transform.rotation.y = q[1]
          t_ref.transform.rotation.z = q[2]
          t_ref.transform.rotation.w = q[3]
          cv2.circle(cv_img_cont, (cX,cY), 3, (255,0,0), -1)
          cv2.line(cv_img_cont, (cX,cY), (cX+int(100*np.cos(euler_z_refined)), cY+int(100*np.sin(euler_z_refined))), (0, 0, 255), thickness=2)
          cv2.line(cv_img_cont, (cX,cY), (cX+int(100*np.cos(euler_z_refined+(np.pi/2))), cY+int(100*np.sin(euler_z_refined+(np.pi/2)))), (0, 255, 0), thickness=2) 
          tfBroadcaster.sendTransform(t_ref)####
        ####

        #print("translation: ", t.transform.translation)
        #print("rotation: ",t.transform.rotation)
        tfBroadcaster.sendTransform(t)
        tfBroadcaster.sendTransform(t_cen)
        
        self.posePub.publish(center_pose)

        
                    
    # visualization
    #cv2.imshow("realsense",cv_img)
    cv2.imshow("canny",cv_img_proc)
    cv2.imshow("detection",cv_img_cont)
    cv2.waitKey(1)

   
 
def main(args):
  rospy.init_node('detector')
  ic = board_detector()
  try:
   rospy.spin()
  except KeyboardInterrupt:
   print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
  main(sys.argv)
