#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import threading
from copy import deepcopy

import rospy
import tf
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Transform, Vector3


class Link:
    def __init__(self, message, locked_axes=(False, False, False), offset=Point()):
        self.message = message
        self.locked_axes = locked_axes
        self.offset = offset

    def apply_offset(self, time=None):
        if not self.locked_axes[0]:
            self.message.transform.translation.x += self.offset.x
        if not self.locked_axes[1]:
            self.message.transform.translation.y += self.offset.y
        if not self.locked_axes[2]:
            self.message.transform.translation.z += self.offset.z
        self.message.header.stamp = time if time is not None else rospy.Time.now()


class TFLinks:
    def __init__(self, update_rate=120):
        self.__links = {}
        self.__broadcaster = tf.TransformBroadcaster()
        self.__listener = tf.TransformListener()

        self.__rate = update_rate
        self.__start_event = threading.Event()
        self.__stop_event = threading.Event()
        self.__thread = threading.Thread(target=self.__run)
        self.__rw_lock = threading.Lock()

    def add_link(self, child, parent, locked_axes, wait_for=3):
        message = TransformStamped()
        message.transform = Transform()
        message.header.frame_id = parent
        message.child_frame_id = child

        try:
            now = rospy.Time.now()
            self.__listener.waitForTransform(parent, child, now, rospy.Duration(wait_for))
            translation, rotation = self.__listener.lookupTransform(parent, child, now)
            message.transform = Transform(translation=Vector3(*translation), rotation=Quaternion(*rotation))
            self.__rw_lock.acquire()
            self.__links[child] = Link(message, locked_axes)
            self.__rw_lock.release()
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr("Could not get transform from '{}' to '{}':\n\tError: {}".format(child, parent, e))

    def set_position_offset(self, child, offset):
        self.__rw_lock.acquire()
        if child not in self.__links:
            rospy.logerr("'{}' not in TF links:\n\t{}".format(child, list(self.__links.keys())))
            self.__rw_lock.release()
            return

        self.__links[child].offset = offset
        self.__rw_lock.release()

    def start(self):
        self.__start_event.set()
        self.__thread.start()
        return self

    def stop(self):
        self.__start_event.clear()
        self.__stop_event.set()
        self.__thread.join()

    def update(self):
        self.__rw_lock.acquire()
        links = deepcopy(self.__links.values())
        self.__rw_lock.release()

        now = rospy.Time.now()
        for link in links:
            link.apply_offset(time=now)
            self.__broadcaster.sendTransformMessage(link.message)

    def __run(self):
        rate = rospy.Rate(self.__rate)
        while not rospy.is_shutdown() and not self.__stop_event.is_set():
            self.update()
            rate.sleep()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


class Linear3dofTFUpdater:
    def __init__(self, parent_frame="linear_3dof_arm_home", camera_frame_2d="linear_3dof_2d_camera_link",
                 camera_frame_3d="linear_3dof_3d_camera_link",
                 gripper_frame="linear_3dof_gripper_link"):

        # Hard coded extrinsic values of 0: parent name, 1: position offset to parent, 2: rotation offset to parent,
        # 3: locked axes that don't change by offsets
        self.child_to_parent = {
            gripper_frame: (parent_frame, (False, False, False)),
            camera_frame_2d: (parent_frame, (False, True, True)),
            camera_frame_3d: (gripper_frame, (True, True, True))
        }

        # Create TF information
        self.tf_links = TFLinks()
        for child, (parent, locked_axes) in self.child_to_parent.items():
            self.tf_links.add_link(child, parent, locked_axes)

        # Updater thread
        self.tf_links.start()

        self.current_position_sub = rospy.Subscriber("/linear_3dof_arm/arm/current_position", Point, self.set_offsets)
        self.gripper_transform = (0.0, 0.0, 0.0)

    def set_offsets(self, arm_position_message):
        offset = Point(arm_position_message.x / 1000, arm_position_message.y / 1000, -arm_position_message.z / 1000)
        for child, info in self.child_to_parent.items():
            self.tf_links.set_position_offset(child, offset)