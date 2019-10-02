#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

from timeit import default_timer as timer

import roslaunch
import rospy
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float32MultiArray, String
from yeet.srv import ArmControllerService, ArmControllerServiceResponse

from linear_3dof_arm.tf_updater import Linear3dofTFUpdater


class ArmController:
    MOVE, PAUSE, RESUME, UNLOCK, HOME, RESET = 0, 1, 2, 3, 4, 5
    MIN_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z = 0, 0, 0, 1065, 500, 510
    OVERFLOW_LIMIT = 10000  # Limit to fix bug with motors going past point

    def __init__(self):
        self.shutdown = False

        self.current_position = Point()
        self.goal_position = Point()
        self.has_homed = False

        self.homing_speed = 50
        self.operation_speed = 150

        self.has_reached_position = False
        self.is_moving = False

        self.allow_interrupt = False
        self.require_homing = True

        # Arm CPP node (manage within this python wrapper)
        rospy.loginfo("Arm Controller: Starting CPP arm control node")
        self.arm_node = roslaunch.core.Node("rasberry_perception", "canopen_singlearm",
                                            "linear_3dof_arm_arm_controller_raw", output="screen")
        self.arm_launch = roslaunch.scriptapi.ROSLaunch()
        self.arm_launch.start()
        self.arm_process = self.arm_launch.launch(self.arm_node)

        # Raw CPP topics
        self.current_position_sub = rospy.Subscriber("/linear_3dof_arm/arm/raw/cur_pos", PoseArray, self.save_position)
        self.reached_position_sub = rospy.Subscriber("/linear_3dof_arm/arm/raw/arm2reached", String, self.save_reached_position)
        self.move_arm_publisher = rospy.Publisher("/linear_3dof_arm/arm/raw/arm2position", Float32MultiArray, queue_size=1)

        # Synchronised python topics with safety logic
        self.movement_subscriber = rospy.Subscriber("/linear_3dof_arm/arm/move_to_position", Point, self.move_to_pose)
        self.reached_position_pub = rospy.Publisher("/linear_3dof_arm/arm/move_to_status", String, queue_size=1)
        self.current_position_pub = rospy.Publisher("/linear_3dof_arm/arm/current_position", Point, queue_size=1)
        self.current_goal_pub = rospy.Publisher("/linear_3dof_arm/arm/current_goal", Point, queue_size=1)

        # Set connection and movement to position timeouts to ensure logic does not get stuck
        self.connection_timeout = 30
        self.arm2pos_timeout = 60
        self.wait_for_rate = 30
        self.__wait_for_connection()

        rospy.loginfo("Arm Controller: Starting arm controller service")
        self.service = rospy.Service('linear_3dof_arm_arm_controller_service', ArmControllerService,
                                     self.arm_service_handler)

        # Home on node launch and set to start position
        if self.require_homing:
            self.send_home()

    def arm_service_handler(self, req):
        if req.home:
            self.send_home()
        elif req.retire:
            self.retire()
        elif req.speed and isinstance(req.speed, int) and 25 <= req.speed <= 400:
            self.operation_speed = req.speed
        else:
            self.send_move(req.x, req.y, req.z)

        return ArmControllerServiceResponse(self.current_position.x, self.current_position.y, self.current_position.z)

    def save_position(self, message):
        self.current_position = message.poses[0].position

        if self.current_position.x >= ArmController.OVERFLOW_LIMIT:
            self.current_position.x = -1
        if self.current_position.y >= ArmController.OVERFLOW_LIMIT:
            self.current_position.y = -1
        if self.current_position.z >= ArmController.OVERFLOW_LIMIT:
            self.current_position.z = -1

        self.current_position_pub.publish(self.current_position)

        if self.is_moving:
            self.current_goal_pub.publish(self.goal_position)

    def save_reached_position(self, message):
        self.has_reached_position = True
        self.reached_position_pub.publish(message)

    def __send_message(self, x, y, z, speed, type_code):
        self.goal_position = Point(x, y, z)
        self.is_moving = True
        message = Float32MultiArray()
        message.data = [x, y, z, speed, type_code]
        self.move_arm_publisher.publish(message)
        self.__wait_until_position_reached()
        self.is_moving = False

    def __wait_for_connection(self):
        rate = rospy.Rate(self.wait_for_rate)
        start_time = timer()
        while self.move_arm_publisher.get_num_connections() == 0:
            if timer() - start_time > self.connection_timeout or self.shutdown or not self.arm_process.is_alive():
                rospy.logerr("Arm Controller: " +
                             "Connection to move arm publisher reached timeout in {}s returning control".format(
                                 self.connection_timeout))
                self.shutdown = True
                break
            rate.sleep()

    def __wait_until_position_reached(self):
        rate = rospy.Rate(self.wait_for_rate)
        start_time = timer()
        while not self.has_reached_position:
            if timer() - start_time > self.arm2pos_timeout or self.shutdown or not self.arm_process.is_alive():
                rospy.logerr("Arm Controller: Arm has not reached position '{}' from '{}' in max allotted time {}s "
                             "returning control".format(ArmController.point_to_array(self.goal_position),
                                                        ArmController.point_to_array(self.current_position),
                                                        self.arm2pos_timeout))
                self.shutdown = True
                break
            rate.sleep()

    @staticmethod
    def point_to_array(message):
        return [message.x, message.y, message.z]

    def send_move(self, x, y, z):
        if self.has_homed:
            if not self.is_moving or self.allow_interrupt:
                x, y, z = self.__process_points(x, y, z)
                if ArmController.validate_position_transform(x, y, z):
                    self.__send_message(x, y, z, self.operation_speed, ArmController.MOVE)
                else:
                    rospy.logerr("Arm Controller: Position data ({}, {}, {}) is invalid".format(x, y, z))
            else:  # Not currently working
                rospy.logerr("Arm Controller: Cannot move to ({}, {}, {}) since arm is already moving".format(x, y, z))
        else:
            rospy.logerr("Arm Controller: Cannot move arm before node has moved arm to home position")

    def __process_points(self, x, y, z):
        return x if x != -1 else self.current_position.x, y if y != -1 else self.current_position.y, z if z != -1 else \
            self.current_position.z

    @staticmethod
    def validate_position_transform(x, y, z):
        return (ArmController.MIN_X <= x <= ArmController.MAX_X or x == -1) and \
               (ArmController.MIN_Y <= y <= ArmController.MAX_Y or y == -1) and \
               (ArmController.MIN_Z <= z <= ArmController.MAX_Z or z == -1)

    def move_to_pose(self, message):
        self.send_move(message.x, message.y, message.z)

    def send_pause(self):
        self.__send_message(0, 0, 0, 0, ArmController.PAUSE)

    def send_resume(self):
        self.__send_message(0, 0, 0, 0, ArmController.RESUME)

    def send_unlock(self):
        self.__send_message(0, 0, 0, 0, ArmController.UNLOCK)

    def send_home(self):
        self.__send_message(0, 0, 0, self.homing_speed, ArmController.HOME)
        self.has_homed = True

    def send_reset(self):
        self.__send_message(0, 0, 0, 0, ArmController.RESET)

    def retire(self):
        # Function to safely return to a position to reduce the risk of arm damage
        rospy.loginfo("Arm Controller: Retiring arm to safe position")
        self.send_move((ArmController.MIN_X + ArmController.MAX_X) / 2,
                       (ArmController.MIN_Y + ArmController.MAX_Y) / 2, ArmController.MAX_Z)

    def run(self):
        rate = rospy.Rate(self.wait_for_rate)

        try:
            while not rospy.is_shutdown() and not self.shutdown:
                rate.sleep()
        except Exception as e:
            rospy.logerr("Arm Controller: Unknown exception occurred: {}".format(e.message))
        finally:
            self.shutdown = True
            self.arm_process.stop()

        rospy.loginfo("Arm Controller: Exiting node")


def arm_controller():
    rospy.init_node('linear_3dof_arm_arm_controller_node', anonymous=True)

    # Launch TF updater for the arm
    rospy.loginfo("Arm Controller: Initialising with ''".format())
    p_odom_frame_id = rospy.get_param('~odom_frame_id', "linear_3dof_arm_home")
    p_gripper_frame = rospy.get_param('~gripper_frame', "linear_3dof_gripper_link")
    p_2d_camera_frame = rospy.get_param('~2D_camera_frame', "linear_3dof_2d_camera_link")
    p_3d_camera_frame = rospy.get_param('~3D_camera_frame', "linear_3dof_3d_camera_link")
    rospy.loginfo(
        "Linear3dofTFUpdater: Initialising tf pose updater with frame_id: '{}', camera: '{}', camera '{}' and "
        "gripper: '{}'".format(p_odom_frame_id, p_2d_camera_frame, p_3d_camera_frame, p_gripper_frame))
    pose_updater = Linear3dofTFUpdater(parent_frame=p_odom_frame_id, camera_frame_2d=p_2d_camera_frame,
                                       camera_frame_3d=p_3d_camera_frame, gripper_frame=p_gripper_frame)

    # Launch Arm
    arm_control_node = ArmController()
    arm_control_node.run()


if __name__ == '__main__':
    arm_controller()
