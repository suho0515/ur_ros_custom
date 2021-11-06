#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("result_image", Image)
    self.pub = rospy.Publisher("point", Point, queue_size = 100)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # rectangle detection
    squares = self.find_squares(cv_image)
    cv2.drawContours( cv_image, squares, -1, (0, 255, 0), 3 )

    # center points
    cX, cY = self.find_moments(squares)

    if cX is 0 or cY is 0: return

    #print(cX, cY)
    pt = Point(cX, cY, 0)
    self.pub.publish(pt)

    # rectangle image visualization
    #cv2.imshow('squares', cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def angle_cos(self, p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

  def find_squares(self, img):
    img = cv2.GaussianBlur(img, (5, 5), 0)
    #cv2.imshow("gaussian_blur",img)
    squares = []
    # color detection
    b, g, r = cv2.split(img)
    #cv2.imshow("green", g)

    # threshold
    _retval, bin = cv2.threshold(g, 150, 255, cv2.THRESH_BINARY)
    #cv2.imshow("binary", bin)

    # square detection
    (_, contours, _) = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
        if len(cnt) == 4 and cv2.contourArea(cnt) > 500 and cv2.contourArea(cnt) < 1500 and cv2.isContourConvex(cnt):
            #print(cv2.contourArea(cnt))
            cnt = cnt.reshape(-1, 2)
            max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
            if max_cos < 0.1:
                squares.append(cnt)

    #print(squares)
    return squares

  def find_moments(self, contours):
    cX = 0
    cY = 0
    for i in contours:
      M = cv2.moments(i)
      cX = int(M['m10'] / M['m00'])
      cY = int(M['m01'] / M['m00']) 
    
    return cX, cY
    

def main(args):
  rospy.init_node('vision_node', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)