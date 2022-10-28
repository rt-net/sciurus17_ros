#! /usr/bin/env python3

import rospy
import rosnode
import numpy as np
import cv2
import tf
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_about_axis
from cv_bridge import CvBridge, CvBridgeError


class ArucoMarkerTracker:
    def __init__(self):
        self._bridge = CvBridge()
        self._image_topic = rospy.get_param("/aruco_detector/cam_image_topic")
        self._camera_frame = rospy.get_param("/aruco_detector/camera_frame")
        self._marker_size = rospy.get_param("/aruco_detector/marker_size")

        self._image_sub = rospy.Subscriber(self._image_topic, Image, self._image_callback, queue_size=1)
        self._marker_image_pub = rospy.Publisher("/sciurus17/aruco_marker/result_image", Image, queue_size=1)
        self._marker_rviz_pub = rospy.Publisher("/sciurus17/aruco_marker/marker", Marker, queue_size=10)
        self._marker_tf_pub = tf.TransformBroadcaster()
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self._parameters = cv2.aruco.DetectorParameters_create()

        # カメラRealSenseD435内部パラメータ
        self._camera_matrix = np.array([[629.19151769, 0.0, 310.8877139], 
                                        [0.0, 629.00756084, 237.17740211], 
                                        [0.0, 0.0, 1.0]])
        # カメラRealSenseD435の歪み
        self._distortion_coefficients = np.array([7.66611486e-02, 7.53799175e-01, 4.40661470e-03, -2.99568609e-03, -3.45666050e+00])

    def _image_callback(self, ros_image):
        try:
            input_image = self._bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        output_image = self._detect_aruco_marker(input_image)
        self._marker_image_pub.publish(self._bridge.cv2_to_imgmsg(input_image, "bgr8"))

    def _detect_aruco_marker(self, marker_image):
        # RGB画像からarucoマーカを検出する
        # corners 検出された各マーカの4コーナーピクセル座標(左上, 右上, 右下, 左下)リスト
        # ids 検出されたマーカのidリスト
        # rejected_marker 無効なマーカのリスト
        corners, ids, rejected_marker = cv2.aruco.detectMarkers(marker_image, self._aruco_dict)
        if corners:
            # 検出されたマーカを画像に書き出す
            cv2.aruco.drawDetectedMarkers(marker_image, corners, ids, (0,255,0))

            # カメラのパラメータでマーカの姿勢を推定する
            # カメラ座標系(右手座標系)において
            # 画像の中心(x,y,z)を(0,0,0)
            # 画像の横軸をx, 右側がプラス
            # 画像の縦軸をy, 下側がプラス
            # 画像の法線向きをz, 画像から離れる方向がプラス
            # rvecs カメラから見たマーカの回転ベクトルリスト
            # tvecs カメラから見たマーカの平行移動ベクトルリスト
            rvecs, tvecs, objpoints = cv2.aruco.estimatePoseSingleMarkers(corners, 
                                                                          self._marker_size, 
                                                                          self._camera_matrix, 
                                                                          self._distortion_coefficients)

            for i, corner in enumerate(corners):
                # 推定された各マーカの姿勢を書き出す
                cv2.drawFrameAxes(marker_image, self._camera_matrix, self._distortion_coefficients, rvecs[i], tvecs[i], 0.03)
                # Rviz内で検出されたマーカを表示する
                self._draw_aruco_rviz_marker(ids[i][0], rvecs[i][0], tvecs[i][0])
            return marker_image
        else:
            return marker_image

    def _draw_aruco_rviz_marker(self, marker_id, rvec, tvec):
        aruco_marker = Marker()
        aruco_marker.header.frame_id = self._camera_frame
        aruco_marker.header.stamp = rospy.Time.now() 
        aruco_marker.lifetime = rospy.Duration(0.5)

        aruco_marker.ns = "aruco_marker_" + str(marker_id)
        aruco_marker.id = marker_id
        aruco_marker.type = Marker.CUBE
        aruco_marker.action = Marker.ADD

        aruco_marker.pose.position.x = tvec[0] 
        aruco_marker.pose.position.y = tvec[1] 
        aruco_marker.pose.position.z = tvec[2] 

        # 回転ベクトルを四元数に変換する
        angle = cv2.norm(rvec)
        q = quaternion_about_axis(angle, rvec)

        aruco_marker.pose.orientation.x = q[0]
        aruco_marker.pose.orientation.y = q[1]
        aruco_marker.pose.orientation.z = q[2]
        aruco_marker.pose.orientation.w = q[3]

        aruco_marker.color.r = 0.0
        aruco_marker.color.g = 1.0
        aruco_marker.color.b = 1.0
        aruco_marker.color.a = 1.0
        aruco_marker.scale.x = 0.04
        aruco_marker.scale.y = 0.04
        aruco_marker.scale.z = 0.01
        
        # tfでマーカの位置と姿勢のフレームを出力する
        self._marker_tf_pub.sendTransform((tvec[0], tvec[1], tvec[2]), q,
                                          rospy.Time.now(),
                                          "marker_"+str(marker_id),
                                          self._camera_frame)
        self._marker_rviz_pub.publish(aruco_marker)

if __name__ == "__main__":
    rospy.init_node("aruco_marker_detector")
    aruco_marker_tracker = ArucoMarkerTracker()
    rospy.spin()
