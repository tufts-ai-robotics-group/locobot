#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

# Define a function that takes in an object pose, a translation delta, and an axis
# and returns the pose of the gripper after the translation
# The object pose is a 6-dimensional vector containing the position and orientation of the object
# it takes the Euler angle convention xyz
# The translation delta is a scalar value
# The axis is an integer: 0 for x-axis, 1 for y-axis, 2 for z-axis


def generate_intermediate_pose(object_pose, delta, axis):
    # Convert the orientation from Euler angles to a rotation matrix
    rotation_matrix = R.from_euler('xyz', object_pose[3:6]).as_dcm()

    # Create a homogeneous transformation matrix from the pose
    transformation_matrix_pose = np.eye(4)
    transformation_matrix_pose[0:3, 0:3] = rotation_matrix
    transformation_matrix_pose[0:3, 3] = object_pose[0:3]

    # Define your translation along the specified axis
    translation = np.zeros(3)
    translation[axis] = delta

    # Create a transformation matrix for the translation
    transformation_matrix_translation = np.eye(4)
    transformation_matrix_translation[0:3, 3] = translation

    # Apply the translation to your pose
    new_transformation_matrix = np.dot(transformation_matrix_pose, transformation_matrix_translation)

    # The new pose is given by the last column of the new transformation matrix
    new_pose = new_transformation_matrix[0:3, 3]

    # To get the new orientation, convert the rotation matrix back to Euler angles
    new_rotation_matrix = new_transformation_matrix[0:3, 0:3]
    new_orientation = R.from_dcm(new_rotation_matrix).as_euler('xyz')

    # Now new_pose and new_orientation contain the pose and orientation of the gripper after the translation
    return np.concatenate([new_pose, new_orientation])

# Test the function with a sample object pose and a translation of 0.1 along the x-axis
object_pose = np.array([1, 0.2, 3, 0, 0.8, 0])  # replace with your actual values
delta = 0.1  # replace with your actual value
axis = 0  # 0 for x-axis, 1 for y-axis, 2 for z-axis
intermediate_pose = generate_intermediate_pose(object_pose, delta, axis)
print(intermediate_pose)