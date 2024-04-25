#!/usr/bin/env python3

import sys

# TODO: JANK LOL
sys.path.append('/home/alison/class/eecs442/cv_ws/src/object_reconstruction')

import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import tf2_ros
from utils.SE3 import SE3

import rospy
import sys
import actionlib

import cv2
import time

from sensor_msgs.point_cloud2 import PointCloud2

class Panorama:

    def __init__(self, name):
        self.pc_subscriber = rospy.Subscriber("/zed/left/point_cloud", PointCloud2, self.pc_callback, queue_size=1)
        self.img_subscriber = rospy.Subscriber("/zed/left/image", Image, self.image_callback, queue_size=1)
        self.pc_publisher = rospy.Publisher("/stitched_pc", PointCloud2)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.img_list = []
        self.current_img = None
        self.current_pc = None
        self.arr_pc = None

    def rotate_pc(self, trans_mat : np.ndarray, pc : np.ndarray):
        # rotate the provided point cloud's x, y points by the se3_pose
        
        # remove NaNs and infs from pc
        pc = pc[np.isfinite(pc).all(axis=1)]

        # represent pc in homogenous coordinates
        points = np.hstack((pc[:,0:3], np.ones((pc.shape[0],1))))
        rotated_points = np.matmul(trans_mat, points.T).T
        pc[:,0:3] = np.delete(rotated_points, 3, 1)
        return pc

    def pc_callback(self, msg: PointCloud2):
        self.current_pc = msg

        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        # TODO: dtype hard-coded to float32
        self.arr_pc = np.frombuffer(bytearray(msg.data), dtype=np.float32).reshape(msg.height * msg.width, int(msg.point_step / 4))[0::30,:]

    def image_callback(self, msg: Image):
        self.current_img = cv2.cvtColor(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3), cv2.COLOR_RGBA2RGB)

    def capture_panorama(self, pc_increments : int):

        time.sleep(1)

        try:
            for i in range(3):
                zed_in_base = SE3.transform_matrix(SE3.from_tf_time(self.tf_buffer, "odom", "zed_left_camera_frame")[0])
                break
        except:
            rospy.loginfo("Failed to get transform from odom to zed_base_link")
        # TODO: Don't hardcode or parametrize this?

        stitched_pc = np.empty((0,4), dtype=np.float32)
        
        for i in range(pc_increments):

            # allow gimbal to come to rest
            time.sleep(0.5)
            
            if self.current_img is None:
                pass
            try:
                zed_in_base_current = SE3.transform_matrix(SE3.from_tf_time(self.tf_buffer, "odom", "zed_left_camera_frame")[0])
                tf_diff = np.linalg.inv(zed_in_base) @ zed_in_base_current
                rotated_pc = self.rotate_pc(tf_diff, self.arr_pc)
                stitched_pc = np.vstack((stitched_pc, rotated_pc))
            except:
                pass

            self.img_list.append(np.copy(self.current_img))

        # rospy.loginfo("Creating Panorama using %s images...", str(len(self.img_list)))
        # stitcher = cv2.Stitcher.create()

        # TODO Handle exceptions in stitching and write to relative path
        # status, pano = stitcher.stitch(self.img_list)

        header = Header()
        # construct image msg
        # try:

            # img_msg = Image()
        #     img_msg.header = header
        #     img_msg.height = pano.shape[0]
        #     img_msg.width = pano.shape[1]
        #     img_msg.encoding = "rgb8"
        #     img_msg.data = pano.tobytes()
        #     img_msg.step = len(pano[0]) * 3
        # except:
        #     rospy.loginfo("Failed to create image message")
        #     self._as.set_aborted()
        #     return
        
        # construct pc from stitched
        try:
            pc_msg = PointCloud2()
            pc_msg.width = stitched_pc.shape[0]
            stitched_pc = stitched_pc.flatten()
            header.frame_id = 'base_link'
            pc_msg.header = header
            pc_msg.fields = self.current_pc.fields
            pc_msg.is_bigendian = self.current_pc.is_bigendian
            pc_msg.data = stitched_pc.tobytes()
            pc_msg.height = 1
            pc_msg.point_step = int(len(pc_msg.data) / pc_msg.width)
            pc_msg.is_dense = self.current_pc.is_dense

            while not rospy.is_shutdown():
                self.pc_publisher.publish(pc_msg)
                time.sleep(0.5)
        except:
            # If image succeeds but pc fails, should we set action as succeeded?
            rospy.loginfo("Failed to create point cloud message")
            return

def main():
    rospy.init_node(name="panorama")
    pano = Panorama(rospy.get_name())
    pano.capture_panorama(5)
    rospy.spin()

if __name__ == "__main__":
    sys.exit(main())
