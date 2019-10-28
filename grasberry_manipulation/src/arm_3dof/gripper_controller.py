#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, Point
from grasberry_manipulation.srv import OpenCloseGripper, OpenCloseGripperResponse


class GripperHandler:
    OPEN_X, CLOSE_X = 50.0, 0.0
    SCISSORS_ON, SCISSORS_OFF = 1.0, 0.0
    BOX_RELEASE, BOX_RETRACT = 1.0, 0.0

    def __init__(self):
        self.gripper_pose_pub = rospy.Publisher("/gripper_poses_arm1", Pose, queue_size=1)
        self.diagnostics_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.set_connected)
        self.serial_connection_started = False
        self.diagnostics = None

        self.service = rospy.Service('open_close_gripper', OpenCloseGripper, self.open_close_handler)
        self.__wait_for_connection()

    def set_connected(self, data):
        self.serial_connection_started = True
        self.diagnostics = data

    def __wait_for_connection(self):
        rospy.loginfo("Gripper Control: Gripper control service initialising.")
        rate = rospy.Rate(40)
        rospy.loginfo("Gripper Control:Waiting for Gripper connection")
        while not rospy.is_shutdown() and self.gripper_pose_pub.get_num_connections() == 0:
            rate.sleep()
        rospy.loginfo("Gripper Control: Gripper connected.")

    def open_close_handler(self, req):
        should_open, should_cut, should_release = req.open_gripper, req.cut, req.release_box
        self.gripper_pose_pub.publish(Pose(
            position=Point(x=GripperHandler.OPEN_X if should_open else GripperHandler.CLOSE_X,
                           y=GripperHandler.SCISSORS_ON if req.open_gripper else GripperHandler.SCISSORS_OFF,
                           z=GripperHandler.BOX_RELEASE if req.open_gripper else GripperHandler.BOX_RETRACT)
        ))
        return OpenCloseGripperResponse(True)  # TODO: Currently there is not way to get gripper status


def handle_open_close_gripper(req):
    return OpenCloseGripperResponse(not req.open_gripper)


def gripper_control_node():
    rospy.init_node('linear_arm_3dof_open_close_gripper_server')
    gripper_handler = GripperHandler()
    rospy.spin()


if __name__ == '__main__':
    gripper_control_node()
