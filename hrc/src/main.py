from mmdeploy_python import Detector, PoseDetector
from camera_streams import CamerStreamHandler
import cv2
import numpy as np
import time
import json
import os
import rospy

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion

class HumanDetector():
    def __init__(self):
        self.detector = Detector(model_path='/home/cps/ws/mmdeploy_model/faster-rcnn', device_name='cuda', device_id=0)

    def eval(self, img):
        bboxes, labels, _ = self.detector(img)
        results = list(zip(bboxes, labels))
        results = list(filter(self.__filter, results))
        return results

    @staticmethod
    def draw_bboxs(img, bboxes):
        for bbox in bboxes:
            bbox, _ = bbox
            [left, top, right, bottom]= bbox[0:4].astype(int)
            cv2.rectangle(img, (left, top), (right, bottom), (0, 255, 0))

    def __filter(self, result):
        bbox, label_id = result

        if((bbox[4] > 0.3) and (label_id == 0)):
            return True
        else:
            return False

class HumanPoseDetector():
    def __init__(self):
        self.detector = PoseDetector(model_path='/home/cps/ws/mmdeploy_model/hrnet_w48_coco_fp16_static', device_name='cuda', device_id=0)

    def eval(self, img, human_bboxes):
        results = []
        
        for human_bbox in human_bboxes:
            bbox, _ = human_bbox
            result = self.detector(img, bbox[0:4])
            _, point_num, _ = result.shape
            points = result[:, :, :2].reshape(point_num, 2)
            results.append(points)
            
        return results

    @staticmethod
    def draw_keypoints(img, multi_human_keypoints):
        for single_human_keypoints in multi_human_keypoints:
            for [x, y] in single_human_keypoints.astype(int):
                cv2.circle(img, (x, y), 3, (0, 255, 0), 4)

class Triangulator():
    '''
    Class for triangulation. Needs camera params and 2d kpts from all cameras to triangulate poses.
    '''
    def __init__(self, camera_params):
        self.camera_params = camera_params
        self.base_camera_idx = 0

    def __triangulate_pair(self, camera_idx1, camera_idx2, multiview_poses):
        '''
        Not intended to be used outside this class's scope
        '''
        P1 = self.camera_params[camera_idx1]["P"]
        P2 = self.camera_params[camera_idx2]["P"]

        x1 = multiview_poses[camera_idx1][0][:, 0:2]
        x2 = multiview_poses[camera_idx2][0][:, 0:2]

        joints3dest = cv2.triangulatePoints(P1, P2, x1.T, x2.T)
        joints3dest /= joints3dest[3]

        return joints3dest[0:3].T

    def triangulate(self, multiview_poses):
        '''
        Return mean of 3d poses from all triangulated camera pairs (in metres).
        '''
        s = np.zeros((17, 3))
        for index in range(len(multiview_poses)):
            if index != self.base_camera_idx:
                s += self.__triangulate_pair(self.base_camera_idx, index, multiview_poses)

        return s / (len(multiview_poses) - 1)

def get_camera_params(camera_params_path):
    '''
    Input: Path to camera calibration file
    Returns: Calculates the projection matrices and puts it in camera_params dictionary
    '''
    f = open(camera_params_path)
    camera_params = json.load(f)["cameras"]

    for param in camera_params:
        K = np.array(param["K"], dtype=np.float64)
        R = np.array(param["R"], dtype=np.float64)
        t = np.array(param["t"], dtype=np.float64) / 100

        param["P"] = K @ np.hstack((R, t))

    return camera_params

def coco_to_voxelpose(coco_kpts):
    t = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    coco_kpts = (t @ coco_kpts.T).T

    voxel_kpts = np.zeros((15,3))
    
    voxel_kpts[0] = (coco_kpts[11] + coco_kpts[12]) /2 
    voxel_kpts[1] = coco_kpts[12]
    voxel_kpts[2] = coco_kpts[14]
    voxel_kpts[3] = coco_kpts[16]
    voxel_kpts[4] = coco_kpts[11]
    voxel_kpts[5] = coco_kpts[13]
    voxel_kpts[6] = coco_kpts[15]
    voxel_kpts[7] = coco_kpts[0]
    voxel_kpts[8] = (coco_kpts[5] + coco_kpts[6]) / 2
    voxel_kpts[9] = coco_kpts[6]
    voxel_kpts[10] = coco_kpts[8]
    voxel_kpts[11] = coco_kpts[10]
    voxel_kpts[12] = coco_kpts[5]
    voxel_kpts[13] = coco_kpts[7]
    voxel_kpts[14] = coco_kpts[9]

    return voxel_kpts

def quat_from_axis_angle(xx, yy, zz, theta):
    factor = np.sin(theta/2.0)
    x = xx * factor
    y = yy * factor
    z = zz * factor
    w = np.cos(theta/2.0)
    q = np.array([x, y, z, w])
    q = q/np.linalg.norm(q)
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]

    return quat


def marker_sphere_list(id, points):
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = 'kinect2_camera'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'human'
    marker.id = id
    marker.type = Marker.SPHERE_LIST
    marker.action = 0
    for point in points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]
        marker.points.append(p)

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 2.0
    marker.pose.orientation = quat_from_axis_angle(0.0, 1.0, 0.0, np.pi/7.0)
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    return marker

def marker_line(id, point1, point2):
    p1 = Point()
    p1.x = point1[0]
    p1.y = point1[1]
    p1.z = point1[2]
    p2 = Point()
    p2.x = point2[0]
    p2.y = point2[1]
    p2.z = point2[2]

    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = 'kinect2_camera'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'human'
    marker.id = id
    marker.type = Marker.LINE_LIST
    marker.action = 0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 2.0
    marker.pose.orientation = quat_from_axis_angle(0.0, 1.0, 0.0, np.pi/7.0)
    marker.points = [p1, p2] 
    marker.scale.x = 0.03
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    return marker

def main():
    rospy.init_node('hrc', anonymous=True)

    camera_params = get_camera_params('/home/cps/ws/mmpose/demo/resources/my_body_3d_2/camera_parameters.json.old')
    csh = CamerStreamHandler()
    human_detector = HumanDetector()
    human_2d_pose_detector = HumanPoseDetector()
    triangulator = Triangulator(camera_params)
    marker_pub = rospy.Publisher('/cobot/joints_markers', MarkerArray, queue_size=10)
    marker_array = MarkerArray()
    
    prev = time.time()
    while not rospy.is_shutdown():
        try:
            time.sleep(0.01)
            frames = csh.grab_frames()
            multiview_poses = []
            for view in frames:
                human_detections = human_detector.eval(view)
                human_poses_2d = human_2d_pose_detector.eval(view, human_detections)
                multiview_poses.append(human_poses_2d)
                
            # rospy.loginfo(multiview_poses)
            pose3d = triangulator.triangulate(multiview_poses)
            pose3d = coco_to_voxelpose(pose3d)

            pose3d = triangulator.triangulate(multiview_poses)
            pose3d = coco_to_voxelpose(pose3d)
            marker_array = MarkerArray()
            marker_array.markers.append(marker_sphere_list(0, pose3d))
            marker_array.markers.append(marker_line(1, pose3d[1], pose3d[0]))
            marker_array.markers.append(marker_line(2, pose3d[2], pose3d[1]))
            marker_array.markers.append(marker_line(3, pose3d[3], pose3d[2]))
            marker_array.markers.append(marker_line(4, pose3d[4], pose3d[0]))
            marker_array.markers.append(marker_line(5, pose3d[5], pose3d[4]))
            marker_array.markers.append(marker_line(6, pose3d[6], pose3d[5]))
            marker_array.markers.append(marker_line(7, pose3d[7], pose3d[8]))
            marker_array.markers.append(marker_line(8, pose3d[9], pose3d[8]))
            marker_array.markers.append(marker_line(9, pose3d[10], pose3d[9]))
            marker_array.markers.append(marker_line(10, pose3d[11], pose3d[10]))
            marker_array.markers.append(marker_line(11, pose3d[12], pose3d[8]))
            marker_array.markers.append(marker_line(12, pose3d[13], pose3d[12]))
            marker_array.markers.append(marker_line(13, pose3d[14], pose3d[13]))
            marker_array.markers.append(marker_line(14, pose3d[8], pose3d[0]))

            marker_pub.publish(marker_array)
            
            time.sleep(0.01) # Sleep is required for giving csh to stream properly

            # rospy.loginfo(f'FPS: {1/(time.time() - prev)}')
            prev = time.time()
            
            
        except Exception as e:
            rospy.loginfo(e)

if __name__ == "__main__":
    main()