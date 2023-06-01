import rospy
import message_filters
import cv_bridge
from threading import Lock

from sensor_msgs.msg import Image

class CamerStreamHandler():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.read_lock = Lock()

        self.__init_messages()
        self.__init_subscribers()

    def __init_messages(self):
        self._kinect2_msg = None
        self._kinect1_msg = None
        self._astra_msg = None

    def __init_subscribers(self):
        self._kinect2_sub = message_filters.Subscriber('/kinect2/hd/image_color', Image)
        self._kinect1_sub = message_filters.Subscriber('kinect1/rgb/image_color', Image)
        self._astra_sub = message_filters.Subscriber('/astra_camera/color/image_raw', Image)

        self._time_sync = message_filters.ApproximateTimeSynchronizer([self._kinect2_sub, self._kinect1_sub, self._astra_sub], 5, 1)
        self._time_sync.registerCallback(self.__sync_callback)

    def __sync_callback(self, kinect2_msg, kinect1_msg, astra_msg):
        print(1)
        self._kinect2_msg = kinect2_msg
        self._kinect1_msg = kinect1_msg
        self._astra_msg = astra_msg

    def grab_frames(self):

        frames = [
            self.bridge.imgmsg_to_cv2(self._kinect2_msg, desired_encoding='bgr8'),
            self.bridge.imgmsg_to_cv2(self._kinect1_msg, desired_encoding='bgr8'),
            self.bridge.imgmsg_to_cv2(self._astra_msg, desired_encoding='rgb8')
        ]

        return frames