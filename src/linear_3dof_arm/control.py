from __future__ import absolute_import, division, print_function

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from rasberry_perception.srv import ArmControllerService, OpenCloseGripper

from linear_3dof_arm.arm_controller import ArmController


class Linear3dofController:
    def __init__(self, arm_enabled=True, gripper_enabled=True, arm_blocking=True):
        self.rate = rospy.Rate(120)
        self.__arm_enabled = arm_enabled
        self.__gripper_enabled = gripper_enabled

        if self.__arm_enabled:
            self.current_position = Point()
            self.has_reached_position = False
            self.has_reached_position_message = None

            # Wait for arm connection
            self.movement_publisher = rospy.Publisher("/linear_3dof_arm/arm/move_to_position", Point, queue_size=1)

            rospy.loginfo("Waiting for connection to arm controller")
            while self.movement_publisher.get_num_connections() == 0:
                self.rate.sleep()
            rospy.wait_for_service('linear_3dof_arm_arm_controller_service')
            self.arm_controller_service = rospy.ServiceProxy('linear_3dof_arm_arm_controller_service',
                                                             ArmControllerService)
            rospy.loginfo("Connected to arm controller")

            self.blocking = arm_blocking

            # Subscribe to arm controller topics
            self.reached_position_sub = rospy.Subscriber("/linear_3dof_arm/arm/move_to_status", String,
                                                         self.__save_reached_position)
            self.current_position_sub = rospy.Subscriber("/linear_3dof_arm/arm/current_position", Point,
                                                         self.__save_position,
                                                         queue_size=1)

        if self.__gripper_enabled:
            # Wait for gripper connection
            rospy.loginfo("Waiting for connection to gripper control service")
            rospy.wait_for_service('open_close_gripper')
            self.gripper_control_service = rospy.ServiceProxy('open_close_gripper', OpenCloseGripper)
            rospy.loginfo("Connected to gripper controller")

    def __save_position(self, message):
        self.current_position = message

    def __save_reached_position(self, message):
        self.has_reached_position = True
        self.has_reached_position_message = message

    def __wait_until_position_reached(self):
        self.has_reached_position = False
        while not self.has_reached_position:
            self.rate.sleep()

    def __arm_do_action(self, x=-1, y=-1, z=-1, speed=0, home=False, retire=False):
        if self.__arm_enabled:
            if -1 not in (x, y, z):
                if not Linear3dofController.verify_move_xyz(x, y, z):
                    return None
            try:
                response = self.arm_controller_service(x, y, z, speed, home, retire)
                if self.blocking and not speed:
                    self.__wait_until_position_reached()
                return response
            except rospy.ServiceException as e:
                rospy.loginfo("Arm service call failed: {}".format(e))
        return None

    # TODO: Get status of action to not force multiple actions at the same time
    def __gripper_do_action(self, open_gripper=False, open_scissors=False, release_box=False):
        if self.__gripper_enabled:
            try:
                return self.gripper_control_service(open_gripper, open_scissors, release_box)
            except rospy.ServiceException as e:
                rospy.loginfo("Gripper service call failed: {}".format(e))
        return None

    @staticmethod
    def verify_move_xyz(x, y, z):
        return ArmController.validate_position_transform(x, y, z)

    def move_to_xyz(self, x, y, z):
        return self.__arm_do_action(x, y, z)

    def move_to_point(self, point):
        return self.__arm_do_action(point.x, point.y, point.z)

    def move_to_home(self):
        return self.__arm_do_action(home=True)

    def update_arm_speed(self, speed):
        return self.__arm_do_action(speed=speed)

    def retire_arm(self):
        return self.__arm_do_action(retire=True)

    def open_gripper(self):
        return self.__gripper_do_action(open_gripper=True)

    def close_gripper(self):
        return self.__gripper_do_action(open_gripper=False)

    def open_gripper_scissors(self):
        return self.__gripper_do_action(open_scissors=True)

    def close_gripper_scissors(self):
        return self.__gripper_do_action(open_scissors=False)

    def open_gripper_box(self):
        return self.__gripper_do_action(release_box=True)

    def close_gripper_box(self):
        return self.__gripper_do_action(release_box=False)

    def set_gripper_status(self, open_gripper=False, open_scissors=False, release_box=False):
        return self.__gripper_do_action(open_gripper, open_scissors, release_box)
