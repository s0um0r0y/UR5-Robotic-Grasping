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
