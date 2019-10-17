#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import cv2
import numpy as np
import ros_numpy
import rospy
from sensor_msgs.msg import Image

from linear_3dof_arm.control import Linear3dofController


class InteractiveControl:
    def __init__(self):
        self.help_text = "Controls:\n  Q/E: Y axis +/-\n  W/S: Z axis +/-\n  A/D: X axis +/-\n " \
                         " -/+: Increase/Decrease slide\nH: home all axis\nO: Open Gripper\nP: Close Gripper" \
                         "\nR: Retire arm to safety"
        self.last_message = ""

        self.window_name = "Control Window"
        self.update_amount = 20

        self.keys = None
        self.create_keys()

        self.frame = np.random.random((512, 512))
        rospy.loginfo("Waiting for colour topic linear_3dof_2d_camera/color/image_raw")
        self.colour_sub = rospy.Subscriber("/linear_3dof_2d_camera/color/image_raw", Image, self.save_frame)

        rate = rospy.Rate(30)
        while self.colour_sub.get_num_connections() == 0:
            rate.sleep()

        self.controller = Linear3dofController()

    def save_frame(self, message):
        frame = ros_numpy.numpify(message)[..., ::-1].astype(np.uint8)
        font, font_scale = cv2.FONT_HERSHEY_DUPLEX, 0.7
        y0, dy = 20, cv2.getTextSize(self.help_text, font, font_scale, 2)[1] + 20

        for i, line in enumerate(self.help_text.split('\n')):
            y = y0 + i * dy
            cv2.putText(frame, line, (y0, y), font, font_scale, [0, 0, 0], thickness=2)
            cv2.putText(frame, line, (y0, y), font, font_scale, [255, 255, 255], thickness=1)

        cv2.putText(frame, self.last_message, (y0, frame.shape[0] - 25), font, font_scale, [0, 0, 0], thickness=2)
        cv2.putText(frame, self.last_message, (y0, frame.shape[0] - 25), font, font_scale, [255, 255, 255], thickness=1)
        self.frame = frame

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_GUI_EXPANDED)

        while not rospy.is_shutdown():
            cv2.imshow(self.window_name, self.frame)
            key_pressed = cv2.waitKey(30)

            if key_pressed == 45 and self.update_amount > 0:  # Decrease on -
                self.change_slide(-5)
            if key_pressed == 43 and self.update_amount < 200:  # Increase on +
                self.change_slide(5)

            if key_pressed != -1:
                if key_pressed in self.keys:
                    old_position = np.array([self.controller.current_position.x,
                                             self.controller.current_position.y,
                                             self.controller.current_position.z])
                    new_position = old_position + self.keys[key_pressed]
                    self.text_print("Key '{}' pressed, moving to: '{}'".format(chr(key_pressed), new_position))
                    self.controller.move_to_xyz(*new_position)
                elif key_pressed == 104:
                    self.text_print("Key '{}' pressed, moving all axis home".format(chr(key_pressed)))
                    self.controller.move_to_home()
                elif key_pressed == 114:
                    self.text_print("Key '{}' pressed, retiring gripper to safe position".format(chr(key_pressed)))
                    self.controller.retire_arm()
                elif key_pressed == 111:
                    self.text_print("Key '{}' pressed, opening gripper".format(chr(key_pressed)))
                    self.controller.open_gripper()
                elif key_pressed == 112:
                    self.text_print("Key '{}' pressed, closing gripper".format(chr(key_pressed)))
                    self.controller.close_gripper()

    def create_keys(self):
        self.keys = {
            113: np.array([0, self.update_amount, 0]),  # Q
            119: np.array([0, 0, -self.update_amount]),  # W
            101: np.array([0, -self.update_amount, 0]),  # E
            97: np.array([-self.update_amount, 0, 0]),  # A
            115: np.array([0, 0, self.update_amount]),  # S
            100: np.array([self.update_amount, 0, 0])  # D
        }

    def change_slide(self, amount):
        self.update_amount += amount
        self.create_keys()
        self.text_print("{}creased update (slide) amount to: {}".format("De" if amount < 0 else "In",
                                                                        self.update_amount))

    def text_print(self, message):
        self.last_message = message


def interactive_control():
    rospy.init_node('rasberry_perception_interactive_control', anonymous=True)

    control = InteractiveControl()
    control.run()


if __name__ == '__main__':
    interactive_control()
