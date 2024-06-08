#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
from hand_detection.msg import ImageWithRectangles, Rectangle # type: ignore
from geometry_msgs.msg import Pose, Point, PoseArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import matplotlib.pyplot as plt

class point_get:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb = 0
        self.rgb_get_flag = 0
        self.depth = 0
        self.rectangle = 0
        self.RGBinfo = 0
        self.points = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cam0/k4a/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber('/cam0/k4a/depth_to_rgb/image_raw', Image, self.depth_image_callback,queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/cam0/k4a/rgb/camera_info', CameraInfo, self.info_callback,queue_size=1)
        self.rectangle_sub = rospy.Subscriber('/hand_detection/ImageWithRectangles', ImageWithRectangles, self.rectangles_callback,queue_size=1)
        self.pose_pub = rospy.Publisher('/obstacle/points', PoseArray, queue_size=1)
        

    def info_callback(self, data):
        self.RGBinfo = data

    def image_callback(self, data):
        self.rgb = data
        self.depth = 0
        self.rectangle = 0

    def depth_image_callback(self, data):
        self.depth = data
        self.make_point()

    def rectangles_callback(self, data):
        self.rectangle = data
        self.make_point()

    def find_median(self, image, x1, y1, x2, y2):
        # 사각형 범위 내의 픽셀 값 추출
        pixel_values = image[y1:y2+1, x1:x2+1].flatten()
        
        # 0인 값과 1000 이상인 값 제외
        valid_values = pixel_values[(pixel_values != 0) & (pixel_values < 1000)]
        
        if len(valid_values) == 0:
            return 0
        
        # 픽셀 값을 정렬
        sorted_pixel_values = np.sort(valid_values)
        
        # 중간 인덱스를 계산
        middle_index = len(sorted_pixel_values) // 2
        
        # 중간 값 계산 (짝수 개수일 경우 두 중앙 값의 평균)
        if len(sorted_pixel_values) % 2 == 0:
            median_value = (sorted_pixel_values[middle_index - 1] + sorted_pixel_values[middle_index]) / 2
        else:
            median_value = sorted_pixel_values[middle_index]
        
        print(median_value)
        return median_value
    def pixel_to_3d_point(self, x, y, depth, K):
            # 깊이 값을 미터 단위로 변환
            depth_m = depth / 1000.0

            fx = K[0]
            fy = K[4]
            cx = K[2]
            cy = K[5]
            
            X = (x - cx) * depth_m / fx
            Y = (y - cy) * depth_m / fy
            Z = depth_m
            
            return [X, Y, Z]

    def make_point(self):
        if(self.depth != 0 and self.rectangle != 0 and self.RGBinfo != 0):
            if(len(self.rectangle.rectangles) != 0):
                rectangleInfo = self.rectangle.rectangles
                poses = PoseArray()
                for rectangle in rectangleInfo:
                    image = self.depth
                    image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
                    depth = self.find_median(image, rectangle.x1,rectangle.y1,rectangle.x2,rectangle.y2)
                    if(depth == 0):
                        return 0
                    width = (rectangle.x1 + rectangle.x2)/2
                    height = (rectangle.y1 + rectangle.y2)/2
                    K =self.RGBinfo.K
                    point = self.pixel_to_3d_point(width, height, depth, K)
                    pose = Pose()
                    pose.position.x = point[0]
                    pose.position.y = point[1]
                    pose.position.z = point[2]
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1

                    poses.poses.append(pose)
                    print(f'Pose: {pose}')

                self.pose_pub.publish(poses)

            self.depth = 0
            self.rectangle = 0
            self.rgb = 0


if __name__ == '__main__':
    rospy.init_node('point_get')
    hd_node = point_get()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Hand Detection Node")
    cv2.destroyAllWindows()
