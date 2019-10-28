#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy

from arm_3dof.control import Linear3dofController


def open_close_gripper():
    rospy.init_node('rasberry_perception_example_open_close_gripper', anonymous=True)

    yeet_controller = Linear3dofController(arm_enabled=False, gripper_enabled=True)

    while not rospy.is_shutdown():
        yeet_controller.open_gripper()
        rospy.sleep(2)
        yeet_controller.close_gripper()
        rospy.sleep(2)


if __name__ == '__main__':
    open_close_gripper()
