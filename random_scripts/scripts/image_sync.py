#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image


kinect2_pub = rospy.Publisher('/kinect2/hd/image_color_sync', Image, queue_size=10)
astra_pub = rospy.Publisher('/astra_camera/color/image_raw_sync', Image, queue_size=10)

def callback(image1, image2):
    global kinect2_pub, astra_pub

    kinect2_pub.publish(image1)
    astra_pub.publish(image2)

def main():
    kinect2_sub = message_filters.Subscriber('/kinect2/hd/image_color', Image)
    astra_sub = message_filters.Subscriber('/astra_camera/color/image_raw', Image)

    time_sync = message_filters.ApproximateTimeSynchronizer([kinect2_sub, astra_sub], 10, 0.05)
    time_sync.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_sync', anonymous=True)
    main()