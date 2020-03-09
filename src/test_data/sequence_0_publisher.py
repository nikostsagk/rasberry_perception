#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com
import pickle

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


def __main():
    import pathlib
    pico_root = (pathlib.Path(__file__).parent / "pico_2019_10_11").resolve()

    if not pico_root.is_dir():
        raise ValueError("Request the download of sequence 0 from Raymond "
                         "http://lcas.lincoln.ac.uk/owncloud/index.php/s/PYEccW0yWvZaSz4/download")

    files = sorted(list(str(s) for s in pico_root.glob("*.pkl")))

    fps = 10
    seconds_limit = 30
    frame_limit = fps * seconds_limit
    print("Truncating files array (n={}) to {} seconds (n={}) for {} fps".format(len(files), seconds_limit, frame_limit,
                                                                                 fps))
    files = files[:frame_limit]

    ram_files = []
    print("Loading files into RAM")
    from tqdm import tqdm

    for file in tqdm(files):
        with open(file, "r") as fh:
            data = pickle.load(fh)
        ram_files.append(data)
        # if len(ram_files) > 5:
        #     break

    rospy.init_node("sequence_0_publisher", anonymous=True)
    hz = rospy.Rate(fps)

    republish_namespace = "sequence_0/"
    image_pub = rospy.Publisher(republish_namespace + "colour/image_raw", Image, queue_size=1)
    image_info_pub = rospy.Publisher(republish_namespace + "colour/camera_info", CameraInfo, queue_size=1)
    depth_pub = rospy.Publisher(republish_namespace + "depth/image_raw", Image, queue_size=1)
    depth_map_pub = rospy.Publisher(republish_namespace + "depth/cmap_raw", Image, queue_size=1)
    depth_info_pub = rospy.Publisher(republish_namespace + "depth/camera_info", CameraInfo, queue_size=1)
    robot_position = rospy.Publisher(republish_namespace + "robot/pose", Pose, queue_size=1)
    sequence_starts = rospy.Publisher(republish_namespace + "info", String, queue_size=1)
    current_node = rospy.Publisher(republish_namespace + "current_node", String, queue_size=1)

    from timeit import default_timer as timer
    iteration = 0

    while not rospy.is_shutdown():
        print("Running iteration {}".format(iteration))
        sequence_starts.publish(String("Sequence 0, iteration {} starting".format(iteration)))
        start = timer()

        for frame_idx, data in enumerate(ram_files):
            if frame_idx >= frame_limit:
                print("Reached frame limit {}>={} for time limit of {} seconds".format(frame_idx, frame_limit,
                                                                                       seconds_limit))
                break
            now = rospy.Time.now()
            data["rgb"]["image"].header.stamp = now
            data["aligned_depth_to_rgb"]["image"].header.stamp = now
            data["aligned_depth_to_rgb"]["intrinsics"].header.stamp = now
            data["aligned_depth_to_rgb"]["colourmap"].header.stamp = now
            data["rgb"]["intrinsics"].header.stamp = now

            image_pub.publish(data["rgb"]["image"])
            image_info_pub.publish(data["rgb"]["intrinsics"])
            depth_pub.publish(data["aligned_depth_to_rgb"]["image"])
            depth_info_pub.publish(data["aligned_depth_to_rgb"]["intrinsics"])
            depth_map_pub.publish(data["aligned_depth_to_rgb"]["colourmap"])
            robot_position.publish(data["localisation"]["robot_pose"])
            if data["localisation"]["current_node"] is not None:
                current_node.publish(data["localisation"]["current_node"])

            hz.sleep()
        print("Iteration {} took {} seconds".format(iteration, timer() - start))
        iteration += 1

    print(files)


if __name__ == "__main__":
    __main()
