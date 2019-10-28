#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

from copy import deepcopy

import rospy

from arm_3dof.control import Linear3dofController


def interactive_control_cli():
    rospy.init_node("rasberry_perception_interactive_control_cli")

    try:
        rasberry_perception_controller = Linear3dofController(arm_enabled=True, gripper_enabled=False)
        action_defaults = {'x': -1, 'y': -1, 'z': -1, 'home': False, 'retire': False}

        while not rospy.is_shutdown():
            command = raw_input("Please enter your command: ")

            if len(command) == 0:
                continue

            action_keys = command.replace(' ', '').split(',')
            prompt_actions = [c.split('=') for c in action_keys]
            prompt_actions = [c for c in prompt_actions if len(c) == 2 and c[0] in action_defaults]
            action = deepcopy(action_defaults)
            for k, v in prompt_actions:
                try:
                    v = int(v)
                except ValueError:
                    v = bool(v)
                action[k] = v

            print("Sending action:", action)
            if action['home']:
                rasberry_perception_controller.move_to_home()
            elif action['retire']:
                rasberry_perception_controller.retire_arm()
            else:
                rasberry_perception_controller.move_to_xyz(action['x'], action['y'], action['z'])
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    interactive_control_cli()
