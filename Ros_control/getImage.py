# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time
# Instantiate CvBridge
bridge = CvBridge()


def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('./Image/fetch.jpeg', cv2_img)
        print("Received an image!")
        time.sleep(2)


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # image_topic = '/head_camera/depth_downsample/image_raw'
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
