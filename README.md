# UR5-Robotic-Grasping
I am working new and novel grasping mechanism with rg2 collaborative gripper on UR5 in GD Naidu robotics lab in VIT Vellore.

# Humanoid Bullet Control
## Description
`humanoid_bullet_control` is a package for [pybullet](https://github.com/bulletphysics/bullet3) that makes it easy to control the [humanoid](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_data/humanoid/humanoid.urdf) urdf model provided with pybullet using human poses. Currently, it supports pose format, output by [voxelpose](https://github.com/microsoft/voxelpose-pytorch) model.

## Getting started
This package depends on [human_voxelpose_model](https://github.com/OmkarKabadagi5823/human_voxelpose_model) which reads the poses output by voxelpose and estimate joint angles. It is not available on pip repositories yet, so you need to install it manually.

# Human Voxelpose Model
## Description
`human_voxelpose_model` is a package for estimating the joint angles from human pose. The input format should be in a format that is similar to the output produced by [voxelpose](https://github.com/microsoft/voxelpose-pytorch) model.

## Voxelpose Output Format
Voxelpose is a Multi-Camera 3D Human Pose Estimation model which estimates 3D poses of multiple people from multiple camera views. The output of voxelpose is a position estimate in 3D-space of 15 human joints which are:
-  0: hip (hip centre)
-  1: r_hip (right hip)
-  2: r_knee (right knee)
-  3: r_foot (right foot)
-  4: l_hip (left hip)
-  5: l_knee (left knee)
-  6: l_foot (left foot)
-  7: nose
-  8: c_shoulder (shoulder centre)
-  9: r_shoulder (right shoulder)
- 10: r_elbow (right elbow)
- 11: r_wrist (right wrist)
- 12: l_shoulder (left shoulder)
- 13: l_elbow (left elbow)
- 14: l_wrist (left wrist)

This is output as `15 x 3` tensor which represents the `x`, `y` and `z` position coordinates of the `15` joints.

### Using HumanPoseModel
The `HumanPoseModel` class provides the implementation of estimating the joint angles from a given pose. There is method exposed in public API which is the `update()` method which takes `np.array` of shape `15 x 3` which represents the position of the 15 joints and updates the joint angles accordingly. `<HumanPoseModel>.rot` dictionary can be used to access the the joint angles which are stored as `scipy.spatial.transform.Rotation`. This allows the user to then read the joint angles in their required format (rotation matrix, euler angles, quaternions or rotation vectors).

#### root frame
`root` frame acts as the base frame for the human model. This frame is centered at the hip, with its x-axis coming out of the hip towards the front with the frame always parallel to the ground plane.

#### Valid keys for HumanPoseModel.rot
- 'w|r'     (root frame in world frame)
- 'w|0'     (hip in world frame)
- 'w|8'     (neck in world frame)
- 'w|9'     (r_shoulder in world frame)
- 'w|12'    (l_shoulder in world frame)
- '9|10'    (r_elbow in r_shoulder frame) # Note that this simply gives the angle in radians as elbow is revolute joint
- '12|13'   (l_elbow in l_shoulder frame) # Note that this simply gives the angle in radians as elbow is revolute joint
- 'w|1'     (r_hip in world frame)
- 'w|4'     (l_hip in world frame)
- '1|2'     (r_knee in r_hip frame) # Note that this simply gives the angle in radians as knee is revolute joint
- '4|5'     (l_knee in l_hip frame) # Note that this simply gives the angle in radians as knee is revolute joint
- 'r|0'     (hip in root frame)
- '0|8'     (neck in hip frame)
- '0|9'     (r_shoulder in hip frame)
- '0|12'    (l_shoulder in hip frame)
- 'r|1'     (r_hip in root frame)
- 'r|4'     (l_hip in root frame)
