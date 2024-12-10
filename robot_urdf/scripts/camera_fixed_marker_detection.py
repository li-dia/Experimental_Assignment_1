#!/usr/bin/env python3

# Ros libraries
import roslib
import rospy
import time
import math

# Python libs
import sys

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros Messages
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo


class MarkerDetectionNode:
    def __init__(self):
        rospy.init_node("marker_detection_node", anonymous=True)

        # Marker ID and position
        self.MarkerIDs = []
        self.CameraInfo = None
        self.TargetID = None
        self.CurrentMarkerID = None
        self.CurrentMarkerCenter = Point()
        self.SearchingForMarkers = True

        # ROS Publishers and Subscribers
        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.ImageCallback)
        self.camera_info_sub = rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.CameraCallback)
        self.marker_id_sub = rospy.Subscriber("/marker_id", Int32, self.MarkerIDCallback)
        self.marker_pos_sub = rospy.Subscriber("/marker_center", Point, self.MarkerCenterCallback)

        self.output_image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.sleep(1)

    def CameraCallback(self, msg):
        # Store the CameraInfo message for later use
        self.CameraInfo = msg

    def MarkerIDCallback(self, msg):
        self.CurrentMarkerID = msg.data

    def MarkerCenterCallback(self, msg):
        self.CurrentMarkerCenter.x = msg.x
        self.CurrentMarkerCenter.y = msg.y


    def ImageCallback(self, msg):
        # Direct conversion to CV2
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0
        
        print("in imagecallback")

        vel_cmd = Twist()

        if self.SearchingForMarkers:
            vel_cmd.angular.z = 0.7
            print("searching for markers")

            if self.CurrentMarkerID not in self.MarkerIDs and self.CurrentMarkerID:
                print("found marker", self.CurrentMarkerID)
                self.MarkerIDs.append(self.CurrentMarkerID)
                self.MarkerIDs = sorted(self.MarkerIDs)

            if len(self.MarkerIDs) == 5:
                print("all markers are detected moving to next step")
                self.SearchingForMarkers = False
                vel_cmd.angular.z = 0


        else:
            if len(self.MarkerIDs) > 0:
                print("setting new target id: ", self.TargetID)
                self.TargetID = self.MarkerIDs[0]
                if self.CameraInfo is not None:
                    cx = self.CameraInfo.K[2]
                    cy = self.CameraInfo.K[5]
                    print("The camera center is: ", cx, ",", cy)

                    if self.TargetID == self.CurrentMarkerID:
                        # Check if the centers are aligned
                        TargetX = self.CurrentMarkerCenter.x
                        TargetY = self.CurrentMarkerCenter.y
                        print("the target center is : ", TargetX, ",", TargetY)

                        # Calculate the horizontal and vertical offsets between the camera center and the marker
                        print("calculating the offset")
                        OffsetX = TargetX - cx
                        OffsetY = TargetY - cy
                        print("Offsetx is ", OffsetX)

                        if abs(OffsetX) < 5:
                            print("offset is less than 5, publishing the image")
                            
                            # Publish the image and pop out the marker from the list
                            cv2.circle(image_np, (int(TargetX), int(TargetY)), 250, (0, 255, 0), 3)

                            Image_msg = CompressedImage()
                            Image_msg.header.stamp = rospy.Time.now()
                            Image_msg.format = "jpeg"
                            Image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

                            self.output_image_pub.publish(Image_msg)
                            self.MarkerIDs.pop(0)
                            vel_cmd.angular.z = 0
                        elif abs(OffsetX) > 5:
                            print("proportional control")
                            # Proportional control to move the robot towards the marker
                            vel_cmd.angular.z = 0.1  # Adjust orientation based on horizontal offset
                            rospy.sleep(0.1)  # Sleep for 100ms to prevent rapid command changes
                    else:
                        # Keep turning
                        print("keep turning to find markers")
                        vel_cmd.angular.z = 0.7
            else:
                vel_cmd.angular.z = 0
                print("All markers processed, stopping.")

        self.vel_cmd.publish(vel_cmd)


def main():
    print("start")
    marker_detection_node = MarkerDetectionNode()

    rospy.spin()


if __name__ == '__main__':
    main()

