#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from hand_detection.msg import ImageWithRectangles, Rectangle # type: ignore
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp

class HandDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cam0/k4a/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/hand_detection/image', Image, queue_size=10)
        self.rectangle_pub = rospy.Publisher('/hand_detection/ImageWithRectangles', ImageWithRectangles, queue_size=10)
        
        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return

        # Convert the BGR image to RGB before processing
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(rgb_image)
        image_with_rectangles_msg = ImageWithRectangles()
        rectangles = []

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                x_min = min([landmark.x for landmark in hand_landmarks.landmark])
                y_min = min([landmark.y for landmark in hand_landmarks.landmark])
                x_max = max([landmark.x for landmark in hand_landmarks.landmark])
                y_max = max([landmark.y for landmark in hand_landmarks.landmark])

                h, w, _ = cv_image.shape
                x_min = int(x_min * w)
                y_min = int(y_min * h)
                x_max = int(x_max * w)
                y_max = int(y_max * h)

                rectangle_msg = Rectangle()
                rectangle_msg.x1 = x_min
                rectangle_msg.y1 = y_min
                rectangle_msg.x2 = x_max
                rectangle_msg.y2 = y_min
                rectangle_msg.x3 = x_max
                rectangle_msg.y3 = y_max
                rectangle_msg.x4 = x_min
                rectangle_msg.y4 = y_max
                rectangles.append(rectangle_msg)

                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            # print(rectangles)
            image_with_rectangles_msg.image = image_message
            image_with_rectangles_msg.rectangles = rectangles
            self.rectangle_pub.publish(image_with_rectangles_msg)
            self.image_pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')

if __name__ == '__main__':
    rospy.init_node('hand_detection_node')
    hd_node = HandDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Hand Detection Node")
    cv2.destroyAllWindows()
