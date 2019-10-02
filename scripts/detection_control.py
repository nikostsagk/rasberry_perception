#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import rospy
from geometry_msgs.msg import Point, PointStamped
from yeet.msg import LabelledImage

from linear_3dof_arm.control import Linear3dofController
from rasberry_perception_pkg.visualisation import MarkerPublisher
import tf


class DetectionControl:
    def __init__(self):
        self.rate = rospy.Rate(30)

        self.tf_listener = tf.TransformListener()

        # Yeet controller node
        self.yeet_controller = Linear3dofController()

        # Visualisation publishers
        self.marker_publisher = MarkerPublisher("/linear_3dof_arm/arm/harvest_markers",
                                                frame_id="/linear_3dof_arm_home", colours=[[0, 0, 1]])

        # TODO: Remove this debug code
        self.test_point = Point(500, 0, 350)
        self.reset()

        self.camera_3d_distance = 200
        self.strawberry_height = 40

        # Set arm to harvest speed
        self.yeet_controller.update_arm_speed(400)

    def validate_harvest_at(self, x, y, z):
        return self.yeet_controller.verify_move_xyz(x, y, z) and \
               self.yeet_controller.verify_move_xyz(x, y, z - self.strawberry_height) and \
               self.yeet_controller.verify_move_xyz(x, y, z + self.camera_3d_distance)

    def reset(self):
        self.yeet_controller.move_to_point(self.test_point)
        rospy.sleep(3)

    def harvest_at(self, x, y, z):
        goal_point = Point(x, y, z)
        goal_point.z += self.camera_3d_distance
        self.yeet_controller.move_to_point(goal_point)
        self.yeet_controller.open_gripper()
        goal_point.z -= self.camera_3d_distance + self.strawberry_height
        self.yeet_controller.move_to_point(goal_point)
        goal_point.z += self.camera_3d_distance + self.strawberry_height
        self.yeet_controller.move_to_point(goal_point)
        self.yeet_controller.close_gripper()

    def harvest(self, labelled_message):
        annotations = labelled_message.annotations
        parent = labelled_message.parent_frame_id
        child = labelled_message.header.frame_id
        time = labelled_message.header.stamp
        header = labelled_message.header

        # Calculate plan from current position
        # TODO: Manual transform to parent at time of ps
        harvest_points = []
        plan = []
        for annotation in annotations:
            bounding_box = annotation.bounding_box
            shape = annotation.description
            x, y, z = (getattr(shape, s) for s in ['x', 'y', 'z'])

            # Use shape description to make point more accurate
            y -= shape.db  # Since 2D depth only sees half the berry

            # Get world x, y, z for motor controller offset at the time the detection was logged
            wp = PointStamped(header=header, point=Point(x, y, z))
            if parent != child:
                try:
                    print('FIX THE ERROR WITH PARENT CHILD TF IS WRONG IN HEADER and MESSAGE')
                    # wp = self.tf_listener.transformPoint(parent, wp)
                except tf.ExtrapolationException as e:
                    print("\tSkipping {}, {}, {} due to exception '{}'".format(x, y, z, e))
                    continue

            arm_x, arm_y, arm_z = wp.point.x * 1000, wp.point.y * 1000, wp.point.z * -1000

            valid_move = self.validate_harvest_at(arm_x, arm_y, arm_z)
            print("\t{}: XYZ ({}, {}, {}) => ({}, {}, {})".format("Valid" if valid_move else "Invalid Move", x, y, z,
                                                                  arm_x, arm_y, arm_z))

            if valid_move:
                harvest_points.append([wp.point.x, wp.point.y, wp.point.z, 0])
                plan.append([arm_x, arm_y, arm_z])

        # Order plan by lowest points first
        plan = sorted(plan, key=lambda p: (p[2]), reverse=True)

        self.marker_publisher.visualise_points(harvest_points, clear=False)

        # Execute plan
        for arm_x, arm_y, arm_z in plan:
            print("\tMoving to {}, {}, {}".format(arm_x, arm_y, arm_z))
            self.harvest_at(arm_x, arm_y, arm_z)

        self.marker_publisher.clear_markers()
        self.reset()
        return

    def run(self):
        try:
            while not rospy.is_shutdown():
                message = rospy.wait_for_message("/rasberry_perception/detection/rgb_bbox", LabelledImage)
                if isinstance(message, LabelledImage):
                    print("Starting Harvest Operation ({}):".format(len(message.annotations)))
                    self.harvest(message)
                self.rate.sleep()
        except KeyboardInterrupt:
            pass


def detection_control():
    rospy.init_node('yeet_detection_control', anonymous=True)

    print("Yeet Detection Control: Initialising with ''".format())

    control = DetectionControl()
    control.run()


if __name__ == '__main__':
    detection_control()
