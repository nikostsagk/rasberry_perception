#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy

from linear_3dof_arm.control import Linear3dofController


def square_arm_movement():
    rospy.init_node('rasberry_perception_example_square_arm_movement', anonymous=True)

    rasberry_perception_controller = Linear3dofController(arm_enabled=True, gripper_enabled=False)

    points = [[400, 400, 510], [400, 400, 0], [600, 200, 200], [600, 400, 400]]
    while not rospy.is_shutdown():
        for p in points:
            rasberry_perception_controller.move_to_xyz(*p)


if __name__ == '__main__':
    square_arm_movement()
