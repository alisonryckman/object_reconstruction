#!/usr/bin/env python3
import numpy as np
import time
import rospy
from scipy.spatial import KDTree
import os
try:
    import open3d as o3d
    visualize = True
except ImportError:
    print('To visualize you need to install Open3D. \n \t>> You can use "$ pip install open3d"')
    visualize = False

from assignment_4_helper import ICPVisualizer, load_point_cloud, view_point_cloud, quaternion_matrix, \
    quaternion_from_axis_angle, load_pcs_and_camera_poses, save_point_cloud

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class ICP:
    def __init__(self):
        self.pc_subscriber = rospy.Subscriber("/zed/left/point_cloud", PointCloud2, self.pc_callback, queue_size=1)
        self.pc_publisher = rospy.Publisher("/stitched_pc", PointCloud2, queue_size=1)
        self.last_recorded_ts = rospy.Time.now()
        self.current_pc = None
        self.stitched_pc = None
        self.first_pc = None

    def transform_point_cloud(self, point_cloud, t, R):
        """
        Transform a point cloud applying a rotation and a translation
        :param point_cloud: np.arrays of size (N, 6)
        :param t: np.array of size (3,) representing a translation.
        :param R: np.array of size (3,3) representing a 3D rotation matrix.
        :return: np.array of size (N,6) resulting in applying the transformation (t,R) on the point cloud point_cloud.
        """
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        # transformed_point_cloud = point_cloud
        point_cloud_copy = np.copy(point_cloud)
        transformed_point_cloud = np.zeros_like(point_cloud_copy)
        point_cloud_distance = (point_cloud_copy[:, 0:3]).transpose()

        transformed_point_cloud_distances = (R@point_cloud_distance) + t[:, np.newaxis]
        transformed_point_cloud[:, 0:3] = transformed_point_cloud_distances.transpose()
        transformed_point_cloud[:, 3:] = point_cloud_copy[:, 3:]
        # ------------------------------------------------
        return transformed_point_cloud


    def merge_point_clouds(self, point_clouds, camera_poses):
        """
        Register multiple point clouds into a common reference and merge them into a unique point cloud.
        :param point_clouds: List of np.arrays of size (N_i, 6)
        :param camera_poses: List of tuples (t_i, R_i) representing the camera i pose.
                - t: np.array of size (3,) representing a translation.
                - R: np.array of size (3,3) representing a 3D rotation matrix.
        :return: np.array of size (N, 6) where $$N = sum_{i=1}^K N_i$$
        """
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        merged_point_cloud = np.zeros((1, 6))
        for i, point_cloud in enumerate(point_clouds):
            transformed_point_cloud = self.transform_point_cloud(point_cloud, camera_poses[i][0], camera_poses[i][1])
            merged_point_cloud = np.concatenate((merged_point_cloud, transformed_point_cloud), axis=0)
        merged_point_cloud = np.delete(merged_point_cloud, 0, axis=0)
        # ------------------------------------------------
        return merged_point_cloud
    
    def pc_callback(self, msg: PointCloud2):
        # only stitch a pc every two seconds
        #print("test")
        if (rospy.Time.now() - self.last_recorded_ts).to_sec() < 15:
            return
        self.last_recorded_ts = rospy.Time.now()
        # extract xyzrgb fields
        # get every tenth point to make the pc sparser
        self.first_pc = msg
        self.current_pc = np.frombuffer(bytearray(msg.data), dtype=np.float32).reshape(msg.height * msg.width, int(msg.point_step / 4))[0::10,:]

        rospy.loginfo(np.shape(self.current_pc))
        if self.stitched_pc is None:
            print("first iteration, no ICP")
            self.stitched_pc = self.filter_point_cloud(self.current_pc)
            self.pc_publisher.publish(self.numpy_to_pointcloud2(self.stitched_pc))
            return

        scene = self.stitched_pc
        image = self.current_pc
        image_filtered = self.filter_point_cloud(image)

        t, R = self.custom_icp(self.pointcloud2_to_numpy(scene), self.pointcloud2_to_numpy(image_filtered), num_iterations=50)
        rotated_pc = self.transform_point_cloud(image_filtered, t, R)
        self.stitched_pc = np.vstack((self.stitched_pc, rotated_pc))
        if (np.shape(self.stitched_pc)[0] >= 60000):
            rows_to_keep = np.random.choice([True, False], size=self.stitched_pc.shape[0], p=[0.5, 0.5])
            self.stitched_pc = self.stitched_pc[rows_to_keep]
        # ROS_pc = self.numpy_to_pointcloud2(self.stitched_pc)
        print("publishing")
        self.pc_publisher.publish(self.numpy_to_pointcloud2(self.stitched_pc))


    def find_closest_points(self, point_cloud_A, point_cloud_B):
        """
        Find the closest point in point_cloud_B for each element in point_cloud_A.
        :param point_cloud_A: np.array of size (n_a, 6)
        :param point_cloud_B: np.array of size (n_b, 6)
        :return: np.array of size(n_a,) containing the closest point indexes in point_cloud_B
                for each point in point_cloud_A
        """
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        point_cloud_B_dists = np.copy(point_cloud_B[:, 0:3])
        point_cloud_A_dists = np.copy(point_cloud_A[:, 0:3])
        tree = KDTree(point_cloud_B_dists)
        #for index, point in enumerate(point_cloud_A_dists):
        data, index = tree.query(point_cloud_A_dists)

        closest_points_indices = index
        # ------------------------------------------------
        return closest_points_indices


    def find_best_transform(self, point_cloud_A, point_cloud_B):
        """
        Find the transformation 2 corresponded point clouds.
        Note 1: We assume that each point in the point_cloud_A is corresponded to the point in point_cloud_B at the same location.
            i.e. point_cloud_A[i] is corresponded to point_cloud_B[i] forall 0<=i<N
        :param point_cloud_A: np.array of size (N, 6) (scene)
        :param point_cloud_B: np.array of size (N, 6) (model)
        :return:
             - t: np.array of size (3,) representing a translation between point_cloud_A and point_cloud_B
             - R: np.array of size (3,3) representing a 3D rotation between point_cloud_A and point_cloud_B
        Note 2: We transform the model to match the scene.
        """
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        point_cloud_A_dists = np.copy(point_cloud_A[:, 0:3])
        point_cloud_B_dists = np.copy(point_cloud_B[:, 0:3])

        muA = np.mean(point_cloud_A_dists, axis=0)
        muB = np.mean(point_cloud_B_dists, axis=0)

        dim = point_cloud_A_dists.shape[1]
        # W = np.zeros((3,3))
        # for i in range(np.shape(point_cloud_A_dists)[0]):
        #     W += point_cloud_A_dists[i,:]-muA @ (point_cloud_B_dists[i,:]-muB).transpose()
        point_cloud_A_dists -= muA
        point_cloud_B_dists -= muB

        W = np.zeros((dim, dim))
        for i in range(point_cloud_A_dists.shape[0]):
            W += point_cloud_A_dists[i].reshape(dim, 1) @ point_cloud_B_dists[i].reshape(1, dim)

        U, D, V_T = np.linalg.svd(W)
        R_star = U @ V_T
        t_star = muA.reshape(dim, 1) - (R_star @ muB.reshape(dim, 1))
        t = t_star.reshape((dim,))
        R = R_star
        # ------------------------------------------------
        return t, R


    def icp_step(self, point_cloud_A, point_cloud_B, t_init, R_init):
        """
        Perform an ICP iteration to find a new estimate of the pose of the model point cloud with respect to the scene pointcloud.
        :param point_cloud_A: np.array of size (N_a, 6) (scene)
        :param point_cloud_B: np.array of size (N_b, 6) (model)
        :param t_init: np.array of size (3,) representing the initial transformation candidate
                        * It may be the output from the previous iteration
        :param R_init: np.array of size (3,3) representing the initial rotation candidate
                        * It may be the output from the previous iteration
        :return:
            - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
            - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
            - correspondences: np.array of size(n_a,) containing the closest point indexes in point_cloud_B
                for each point in point_cloud_A
        """
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        scene = np.copy(point_cloud_A)
        model = np.copy(point_cloud_B)
        model_tf = self.transform_point_cloud(model, t_init, R_init)
        closest_points = self.find_closest_points(scene, model_tf)
        print("stepped")
        translation_update, rotation_update = self.find_best_transform(scene, model_tf[closest_points])

        R = rotation_update @ R_init
        t = ((rotation_update@t_init) + translation_update)
        correspondences = closest_points
        # ------------------------------------------------
        return t, R, correspondences


    def icp(self, point_cloud_A, point_cloud_B, num_iterations=50, t_init=None, R_init=None, visualize=False):
        """
        Find the
        :param point_cloud_A: np.array of size (N_a, 6) (scene)
        :param point_cloud_B: np.array of size (N_b, 6) (model)
        :param num_iterations: <int> number of icp iteration to be performed
        :param t_init: np.array of size (3,) representing the initial transformation candidate
        :param R_init: np.array of size (3,3) representing the initial rotation candidate
        :param visualize: <bool> Whether to visualize the result
        :return:
             - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
             - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
        """
        if t_init is None:
            t_init = np.zeros(3)
        if R_init is None:
            R_init = np.eye(3)
        if visualize:
            vis = ICPVisualizer(point_cloud_A, point_cloud_B)
        t = t_init
        R = R_init
        correspondences = None  # Initialization waiting for a value to be assigned
        if visualize:
            vis.view_icp(R=R, t=t)
        print("icpeeing")
        for i in range(num_iterations):
            # ------------------------------------------------
            # FILL WITH YOUR CODE
            point_cloud_B_copy = np.copy(point_cloud_B)
            t, R, correspondences = self.icp_step(point_cloud_A, point_cloud_B_copy, t, R)
            # ------------------------------------------------
            if visualize:
                vis.plot_correspondences(correspondences)   # Visualize point correspondences
                time.sleep(0.1)  # Wait so we can visualize the correspondences
                vis.view_icp(R, t)  # Visualize icp iteration
        print("done")
        return t, R


    def filter_point_cloud(self, point_cloud):
        """
        Remove unnecessary point given the scene point_cloud.
        :param point_cloud: np.array of size (N,6)
        :return: np.array of size (n,6) where n <= N
        """
        # ------------------------------------------------
        # FILL WITH YOUR CODE
        input_point_cloud = np.copy(point_cloud)
        xmax = ymax = 0.6
        zmax = 5
        xmin = ymin = -0.6
        zmin = -5


        #basic position filter
        input_point_cloud = input_point_cloud[input_point_cloud[:, 0] <= xmax, :]
        input_point_cloud = input_point_cloud[input_point_cloud[:, 0] >= xmin, :]
        input_point_cloud = input_point_cloud[input_point_cloud[:, 1] <= ymax, :]
        input_point_cloud = input_point_cloud[input_point_cloud[:, 1] >= ymin, :]
        input_point_cloud = input_point_cloud[input_point_cloud[:, 2] <= zmax, :]
        input_point_cloud = input_point_cloud[input_point_cloud[:, 2] >= zmin, :]
        filtered_data = input_point_cloud
        #color based filter
        perfect_yellow = np.array([1,1,0])
        # leeway_percentage = 70
        # max_yellow_threshold = perfect_yellow * (leeway_percentage/100)
        # min_yellow_threshold = perfect_yellow * (1-(leeway_percentage/100))
        # r_mid = 0.8
        # r_diff = 0.2
        # g_mid = 0.8
        # g_diff = 0.2
        # b_mid = 0.1
        # b_diff = 0.1
        #
        # min_yellow_threshold = np.array([r_mid-r_diff, g_mid-g_diff, b_mid-b_diff])
        # max_yellow_threshold = np.array([r_mid + r_diff, g_mid + g_diff, b_mid + b_diff])
        #
        # filtered_data = input_point_cloud[
        #     (input_point_cloud[:, 3] >= min_yellow_threshold[0]) & (input_point_cloud[:, 3] <= max_yellow_threshold[0]) &
        #     (input_point_cloud[:, 4] >= min_yellow_threshold[1]) & (input_point_cloud[:, 4] <= max_yellow_threshold[1]) &
        #     (input_point_cloud[:, 5] >= min_yellow_threshold[2]) & (input_point_cloud[:, 5] <= max_yellow_threshold[2])
        #     ]

        # ------------------------------------------------
        return filtered_data


    def custom_icp(self, point_cloud_A, point_cloud_B, num_iterations=50, t_init=None, R_init=None, visualize=False):
        """
            Find the
            :param point_cloud_A: np.array of size (N_a, 6) (scene)
            :param point_cloud_B: np.array of size (N_b, 6) (model)
            :param num_iterations: <int> number of icp iteration to be performed
            :param t_init: np.array of size (3,) representing the initial transformation candidate
            :param R_init: np.array of size (3,3) representing the initial rotation candidate
            :param visualize: <bool> Whether to visualize the result
            :return:
                 - t: np.array of size (3,) representing a translation estimate between point_cloud_A and point_cloud_B
                 - R: np.array of size (3,3) representing a 3D rotation estimate between point_cloud_A and point_cloud_B
        """
        # ------------------------------------------------

        t, R = self.icp(point_cloud_A, point_cloud_B, num_iterations=num_iterations, t_init=t_init, R_init=R_init, visualize=visualize)
        # ------------------------------------------------
        return t, R

    def numpy_to_pointcloud2(self, points):
        pc_msg = PointCloud2()
        header = Header()
        pc_msg.width = points.shape[0]
        stitched_pc = points.flatten()
        header.frame_id = 'base_link'
        pc_msg.header = header
        pc_msg.fields = self.first_pc.fields
        pc_msg.is_bigendian = self.first_pc.is_bigendian
        pc_msg.data = stitched_pc.tobytes()
        pc_msg.height = 1
        pc_msg.point_step = int(len(pc_msg.data) / pc_msg.width)
        pc_msg.is_dense = self.first_pc.is_dense
        pc_msg.data = (points.flatten()).tobytes()
        return pc_msg

    def pointcloud2_to_numpy(self, points):
        points = points.view(np.uint32)
        rgba_value = points[:, 3]
        r_channel = ((rgba_value >> 24) & 0xFF).reshape(-1, 1)  # Shift right by 24 bits and take the last 8 bits
        g_channel = ((rgba_value >> 16) & 0xFF).reshape(-1, 1)  # Shift right by 16 bits and take the last 8 bits
        b_channel = ((rgba_value >> 8) & 0xFF).reshape(-1, 1)  # Shift right by 8 bits and take the last 8 bits
        a_channel = (rgba_value & 0xFF).reshape(-1, 1)  # Take the last 8 bits for the alpha channel

        return np.hstack((points[:, 0:4], r_channel, g_channel, b_channel)).view(np.float32)

if __name__ == '__main__':
    rospy.init_node(name="icp_node")
    rospy.loginfo("Start")
    icp_node = ICP()
    rospy.spin()
