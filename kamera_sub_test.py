#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        print("seq",msg.header.seq)

        cv2.imshow("frame", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("finished.")

def image_sub():
        rospy.init_node('cv_stream', anonymous=False)
        sub = rospy.Subscriber('line', Image, callback)
        rospy.spin()
	
        cv2.destoryAllWindows()

if __name__ == '__main__':
        image_sub()
