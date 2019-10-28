#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
from geometry_msgs.msg import Point, PointStamped
from rasberry_perception.msg import ImageDetections
from rasberry_perception.msg import HarvestDetections

from arm_3dof.control import Linear3dofController
from rasberry_perception_pkg.visualisation import MarkerPublisher

import tf
import numpy as np


class HarvestingStrategy:
    def __init__(self):
        pass


class DetectionControl:
    def __init__(self):
        self.rate = rospy.Rate(30)

        self.tf_listener = tf.TransformListener()

        self.controller = Linear3dofController()

        # TODO: Remove this debug code
        self.test_point = Point(500, 0, 500)
        self.reset()

        self.marker_publisher = MarkerPublisher("/detection/picking_markers",
                                                frame_id="/linear_arm_3dof_home", colours=[[0, 0, 1]])


        self.cluster_radius = 100
        self.strawberry_height = 125

        # Set arm to harvest speed
        self.controller.update_arm_speed(200)

    def validate_harvest_at(self, x, y, z):
        if not self.controller.verify_move_xyz(x, y, z):
            print("Move outside of the controller bounds")
            return False
        elif not (self.controller.verify_move_xyz(x, y, z - self.strawberry_height) and
                  self.controller.verify_move_xyz(x, y, z + self.cluster_radius)):
            print("Move is outside the needed room (strawberry height and cluster radius")
            return False
        return True

    def reset(self):
        self.controller.move_to_point(self.test_point)
        rospy.sleep(3)

    def harvest_at(self, x, y, z):
        goal_point = Point(x, y, z)
        goal_point.z += self.cluster_radius
        self.controller.move_to_point(goal_point)
        self.controller.open_gripper()
        goal_point.z -= self.cluster_radius + self.strawberry_height
        self.controller.move_to_point(goal_point)
        goal_point.z += self.cluster_radius + self.strawberry_height
        self.controller.move_to_point(goal_point)
        self.controller.close_gripper()

    def move_at(self, x, y, z):
        goal_point = Point(x, y, z)
        self.controller.move_to_point(goal_point)
        self.controller.open_gripper()
        self.controller.close_gripper()

    def harvest(self, labelled_message):
        difference = (rospy.Time.now() - labelled_message.header.stamp).to_sec()
        print("Starting harvest for {} objects, messaged received {} seconds ago.".format(
            len(labelled_message.bounding_boxes), difference))

        harvest_points = []
        plan = []

        for b_box in labelled_message.bounding_boxes:
            x = b_box.x
            y = b_box.y
            z = b_box.z * -1
            actual = (260, 430, 225)
            print(b_box.y,b_box.z,b_box.x)
            predicted = (60.8602319055, 367.117259722, 410.342532397)
            print("\tDetection Parameters: {}".format(b_box))
            arm_x, arm_y, arm_z = x * 1000, y * 1000, z * 1000

            valid_move = True #self.validate_harvest_at(arm_x, arm_y, arm_z)

            print("\t{}: XYZ ({}, {}, {}) => ({}, {}, {})".format("Valid" if valid_move else "Invalid Move", x, y, z,
                                                                  arm_x, arm_y, arm_z))
            harvest_points.append([b_box.x, b_box.y, b_box.z, 0])

            if valid_move:
                plan.append([arm_x, arm_y, arm_z])

        # Order plan by lowest points first
        plan = sorted(plan, key=lambda p: (p[2]), reverse=True)

        self.marker_publisher.visualise_points(harvest_points, clear=False)

        # Execute plan
        for pm_x, pm_y, pm_z in plan:
            print("\tMoving to {}, {}, {}".format(pm_x, pm_y, pm_z))
            # self.move_at(pm_x, pm_y, pm_z)

        self.reset()
        return

    def run(self):
        try:
            rospy.sleep(2)

            while not rospy.is_shutdown():
                # Ensure it's always the latest message
                threshold = 2
                difference = threshold
                message = None
                while difference >= threshold:
                    message = rospy.wait_for_message("/detection/predictions_points", HarvestDetections)
                    difference = (rospy.Time.now() - message.header.stamp).to_sec()
                    print("Got Message from {} seconds ago{}".format(difference,
                                                                     " (Skipping)" if difference >= threshold else ""))
                self.harvest(message)
                print("Sleeping for 1 seconds to ensure berries are stable")
                rospy.sleep(1)
                self.rate.sleep()
        except KeyboardInterrupt:
            pass


def detection_control():
    rospy.init_node('yeet_detection_control', anonymous=True)

    print("Detection Control: Initialising with ''".format())

    control = DetectionControl()
    control.run()


if __name__ == '__main__':
    detection_control()
