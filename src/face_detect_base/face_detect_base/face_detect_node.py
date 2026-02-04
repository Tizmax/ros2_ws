#!/usr/bin/python

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image,RegionOfInterest
# TODO: Add custom message here
from region_of_interest_message.msg import Faces

from cv_bridge import CvBridge
import cv2

class FaceDetect(Node):
    def __init__(self):
        super().__init__("face_detect")

        self.min_size = (10, 10)
        self.image_scale = 2
        self.haar_scale = 1.2
        self.min_neighbors = 2
        self.haar_flags = 0
        self.display = True

        self.opencv_dir = '/usr/share/opencv4/haarcascades/';

        self.face_cascade = cv2.CascadeClassifier(self.opencv_dir + 'haarcascade_frontalface_default.xml')
        if self.face_cascade.empty():
            print("Could not find face cascade")
            sys.exit(-1)
        self.eye_cascade = cv2.CascadeClassifier(self.opencv_dir + 'haarcascade_eye.xml')
        if self.eye_cascade.empty():
            print("Could not find eye cascade")
            sys.exit(-1)
        self.br = CvBridge()
        self.declare_parameter('display', True)
        self.declare_parameter('eyes', True)
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        self.detect_eyes = self.get_parameter('eyes').get_parameter_value().bool_value
        # TODO: Create custom publisher here
        self.pub_faces_roi = self.create_publisher(Faces, "/detected_faces_roi", 10)
        self.pub_faces_img = self.create_publisher(Image, "/detected_faces_image", 10)
        
        self.sub = self.create_subscription(Image,"~/image", self.detect_and_draw, 1)

    def detect_and_draw(self,imgmsg):
        img = self.br.imgmsg_to_cv2(imgmsg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 3)

        # TODO: collect detected faces into a list of region of interest inside a custom message
        detected_faces = Faces()

        for (x,y,w,h) in faces:
            if self.display:
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]

            roi = RegionOfInterest()
            roi.x_offset = int(x)
            roi.y_offset = int(y)
            roi.width = int(w)
            roi.height = int(h)
            detected_faces.rois.append(roi)

            if self.detect_eyes:
                eyes = self.eye_cascade.detectMultiScale(roi_gray)
                if self.display:
                    for (ex,ey,ew,eh) in eyes:
                        cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

        # TODO: publish detected region of interest using self.pub
        self.pub_faces_roi.publish(detected_faces)

        detected_img = self.br.cv2_to_imgmsg(img, "bgr8")
        detected_img.header = imgmsg.header
        self.pub_faces_img.publish(detected_img)
        
        if self.display:
            cv2.imshow('img',img)
            cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)

    face_detect = FaceDetect()

    rclpy.spin(face_detect)

    face_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
