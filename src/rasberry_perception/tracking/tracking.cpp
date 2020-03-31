/**
 * ROS Node used to interact with baysian tracker
 * based on bayes_people_tracker <https://github.com/strands-project/strands_perception_people/>
 * by Christian Dondrup <cdondrup@lincoln.ac.uk>, modified to use 3D cartesian co-ordinates as default
 *
 *
 * @author Raymond Kirk
 * @Version 0.1
 * @Organization University of Lincoln
 * @Status Development
 * @Copyright The MIT License (MIT)
 * @Date March 2020
 *
 */


#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/console.h>
#include <ros/time.h>

#include <bayes_tracking/BayesFilter/bayesFlt.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <XmlRpc.h>

#include "rasberry_perception/tracking/tracking.hpp"


Tracking::Tracking() : detect_seq(0), marker_seq(0) {
    ros::NodeHandle n;

    this->listener = new tf::TransformListener();

    this->startup_time = ros::Time::now().toSec();
    this->startup_time_str = num_to_str<double>(startup_time);

    // Declare variables that can be modified by launch file or command line.
    std::string pub_topic_results;
    std::string pub_topic_results_array;
    std::string pub_topic_detection_results;
    std::string pub_topic_detection_results_array;
    std::string pub_topic_pose_results;
    std::string pub_topic_pose_results_array;
    std::string pub_topic_marker_array;
    std::string pub_topic_detection_marker_array;
    std::string reset_on_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_nh("~");
    private_nh.param("target_frame", target_frame, std::string("sequence_colour_frame"));
    private_nh.param("results", pub_topic_results, std::string("/rasberry_perception/tracking/results"));
    private_nh.param("results_array", pub_topic_results_array, std::string("/rasberry_perception/tracking/results_array"));
    private_nh.param("detection_results", pub_topic_detection_results, std::string("/rasberry_perception/tracking/detection_results"));
    private_nh.param("detection_results_array", pub_topic_detection_results_array, std::string("/rasberry_perception/tracking/detection_results_array"));
    private_nh.param("pose_results", pub_topic_pose_results, std::string("/rasberry_perception/tracking/pose"));
    private_nh.param("pose_results_array", pub_topic_pose_results_array, std::string("/rasberry_perception/tracking/pose_array"));
    private_nh.param("marker_array", pub_topic_marker_array, std::string("/rasberry_perception/tracking/marker_array"));
    private_nh.param("detection_marker_array", pub_topic_detection_marker_array, std::string("/rasberry_perception/tracking/detection_marker_array"));
    private_nh.param("tracker_frequency", this->tracker_frequency, double(30.0));
    private_nh.param("reset_on", reset_on_topic, std::string(""));

    if (parseParams(private_nh) == -1) {
        ROS_FATAL_STREAM("Could not parse config file exiting node.");
        throw reset_exception();
    }

    if (!reset_on_topic.empty()) {
        subs.push_back(
                private_nh.subscribe(reset_on_topic, 1, Tracking::resetCallback)
        );
    }

    this->pub_results = private_nh.advertise<rasberry_perception::Detection>(pub_topic_results, 1);
    this->pub_results_array = private_nh.advertise<rasberry_perception::Detections>(pub_topic_results_array, 1);
    this->pub_detection_results = private_nh.advertise<rasberry_perception::Detection>(pub_topic_detection_results, 1);
    this->pub_detection_results_array = private_nh.advertise<rasberry_perception::Detections>(pub_topic_detection_results_array, 1);
    this->pub_pose_results = private_nh.advertise<rasberry_perception::TaggedPose>(pub_topic_pose_results, 1);
    this->pub_pose_results_array = private_nh.advertise<rasberry_perception::TaggedPoseStampedArray>(pub_topic_pose_results_array, 1);
    this->pub_markers = private_nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array, 1);
    this->pub_detection_markers = private_nh.advertise<visualization_msgs::MarkerArray>(pub_topic_detection_marker_array, 1);

    boost::thread tracking_thread(boost::bind(&Tracking::trackingThread, this));

    ros::MultiThreadedSpinner spinner(8);  //TODO: Replace with ros::spin()
    spinner.spin();
}

int Tracking::parseParams(ros::NodeHandle n) {
    std::string filter;
    n.getParam("filter_type", filter);
    ROS_INFO_STREAM("Found filter type: " << filter);

    float stdLimit = 1.0;
    if (n.hasParam("std_limit")) {
        n.getParam("std_limit", stdLimit);
        ROS_INFO_STREAM("std_limit pruneTracks with " << stdLimit);
    }

    bool prune_named = false;
    if (n.hasParam("prune_named")) {
        n.getParam("prune_named", prune_named);
        ROS_INFO_STREAM("prune_named with " << prune_named);
    }


    if (filter == "EKF") {
        ekf = new SimpleTracking<EKFilter>(stdLimit, prune_named);
    } else if (filter == "UKF") {
        ukf = new SimpleTracking<UKFilter>(stdLimit, prune_named);
    } else if (filter == "PF") {
        pf = new SimpleTracking<PFilter>(stdLimit, prune_named);
    } else {
        ROS_FATAL_STREAM("Filter type is not specified ('" << filter
                                                           << "'). Unable to create the tracker. Please use either EKF, UKF or PF.");
        return -1;
    }

    XmlRpc::XmlRpcValue cv_noise;
    n.getParam("cv_noise_params", cv_noise);
    ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
    if (ekf != nullptr) {
        ekf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"], cv_noise["z"]);
    } else if (ukf != nullptr) {
        ukf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"], cv_noise["z"]);
    } else if (pf != nullptr) {
        pf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"], cv_noise["z"]);
    } else {
        ROS_FATAL_STREAM("no filter configured.");
        return -1;
    }

    ROS_INFO_STREAM("Created " << filter << " based tracker using constant velocity prediction model.");

    XmlRpc::XmlRpcValue detectors;
    n.getParam("detectors", detectors);
    ROS_ASSERT(detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = detectors.begin(); it != detectors.end(); ++it) {
        ROS_INFO_STREAM("Found detector: " << (std::string) (it->first) << " ==> " << detectors[it->first]);
        observ_model_t om_flag;
        double pos_noise_x = .2;
        double pos_noise_y = .2;
        double pos_noise_z = .2;
        int seq_size = 5;
        double seq_time = 0.2;
        association_t association = NN;
        om_flag = CARTESIAN;

        try {
            if (detectors[it->first].hasMember("seq_size"))
                seq_size = (int) detectors[it->first]["seq_size"];
            if (detectors[it->first].hasMember("seq_time"))
                seq_time = (double) detectors[it->first]["seq_time"];
            if (detectors[it->first].hasMember("matching_algorithm")) {
                association = detectors[it->first]["matching_algorithm"] == "NN" ? NN :
                              detectors[it->first]["matching_algorithm"] == "NNJPDA" ? NNJPDA :
                              detectors[it->first]["matching_algorithm"] == "NN_LABELED" ? NN_LABELED
                                                                                         : throw (asso_exception());

                // Set bool to use tags or not
                _use_tags[it->first] =
                        ((std::string) detectors[it->first]["matching_algorithm"]).find("LABELED") != std::string::npos;
            }
            if (detectors[it->first].hasMember("cartesian_noise_params")) { // legacy support
                pos_noise_x = detectors[it->first]["cartesian_noise_params"]["x"];
                pos_noise_y = detectors[it->first]["cartesian_noise_params"]["y"];
                pos_noise_z = detectors[it->first]["cartesian_noise_params"]["z"];
            }
            if (detectors[it->first].hasMember("noise_params")) {
                pos_noise_x = detectors[it->first]["noise_params"]["x"];
                pos_noise_y = detectors[it->first]["noise_params"]["y"];
                pos_noise_z = detectors[it->first]["noise_params"]["z"];
            }
        } catch (XmlRpc::XmlRpcException &e) {
            ROS_FATAL_STREAM("XmlRpc::XmlRpcException: '"
                                     << e.getMessage()
                                     << "'\n"
                                     << "Failed to parse definition for '"
                                     << (std::string) (it->first)
                                     << "'. Check your parameters."
            );
            return -1;
        }

        ROS_INFO("pos_noise_x: %f, pos_noise_y: %f, pos_noise_x: %f, seq_time: %f,  seq_size: %i, fps: %f",
                pos_noise_x, pos_noise_y, pos_noise_z, seq_time, seq_size, this->tracker_frequency);

        try {
            if (ekf != nullptr) {
                ekf->addDetectorModel(
                        it->first,
                        association,
                        om_flag,
                        pos_noise_x, pos_noise_y, pos_noise_z,
                        seq_size, seq_time
                );
            } else if (ukf != nullptr) {
                ukf->addDetectorModel(
                        it->first,
                        association,
                        om_flag,
                        pos_noise_x, pos_noise_y, pos_noise_z,
                        seq_size, seq_time
                );
            } else if (pf != nullptr) {
                pf->addDetectorModel(
                        it->first,
                        association,
                        om_flag,
                        pos_noise_x, pos_noise_y, pos_noise_z,
                        seq_size, seq_time
                );
            }
        } catch (asso_exception &e) {
            ROS_FATAL_STREAM(""
                                     << e.what()
                                     << " "
                                     << detectors[it->first]["matching_algorithm"]
                                     << " is not specified. Unable to add "
                                     << (std::string) (it->first)
                                     << " to the tracker. Please use either NN or NNJPDA as association algorithms."
            );
            return -1;
        } catch (observ_exception &e) {
            ROS_FATAL_STREAM(""
                                     << e.what()
                                     << " "
                                     << detectors[it->first]["observation_model"]
                                     << " is not specified. Unable to add "
                                     << (std::string) (it->first)
                                     << " to the tracker. Please use either CARTESIAN or POLAR as observation models."
            );
            return -1;
        }

        if (detectors[it->first].hasMember("topic") && detectors[it->first]["topic"] != "") {
            subs.push_back(
                    n.subscribe<rasberry_perception::Detections>(
                            (std::string) detectors[it->first]["topic"], 1,
                            boost::bind(&Tracking::detectorCallback, this, _1, it->first)
                    )
            );
        } else if (detectors[it->first].hasMember("tagged_topic") && detectors[it->first]["tagged_topic"] != "") {
            subs.push_back(
                    n.subscribe<rasberry_perception::TaggedPoseStampedArray>(
                            (std::string) detectors[it->first]["tagged_topic"], 1,
                            boost::bind(
                                    &Tracking::detectorCallbackTaggedPoseStampedArray,
                                    this, _1, it->first))
            );
        } else if (detectors[it->first].hasMember("pose_topic") && detectors[it->first]["pose_topic"] != "") {
            subs.push_back(
                    n.subscribe<geometry_msgs::PoseArray>(
                            (std::string) detectors[it->first]["pose_topic"], 1,
                            boost::bind(&Tracking::detectorCallbackPoseArray, this, _1, it->first)
                    )
            );
        } else {
            ROS_FATAL_STREAM(
                    "Invalid topic parameter. Please use a topic of type for " << (std::string) (it->first) <<
                    ":\n\t 1. rasberry_perception::Detections for topic fields\n" <<
                       "\t 2. rasberry_perception::TaggedPoseStampedArray for tagged_topic fields\n" <<
                       "\t 3. geometry_msgs::PoseArray for pose_topic fields\n"
            );
            return -1;
        }
    }

    // All of the configs are valid return 0 to start tracker tread
    return 0;
}

template<typename K, typename V>
static map<V, K> reverse_map(const map<K, V>& m) {
    map<V, K> r;
    for (const auto& kv : m)
        r[kv.second] = kv.first;
    return r;
}

void Tracking::trackingThread() {
    ros::Rate fps(tracker_frequency);
    double time_sec = 0.0;

    while (ros::ok()) {
        std::map<long, std::string> tags;
        try {
            // Results from the tracked objects thread (detectorCallback)
            track_results tracks;
            if (ekf != nullptr) {
                tracks = ekf->track(&time_sec, tags);
            } else if (ukf != nullptr) {
                tracks = ukf->track(&time_sec, tags);
            } else if (pf != nullptr) {
                tracks = pf->track(&time_sec, tags);
            }

            std::map<long, std::vector<rasberry_perception::Detection>> detections = std::get<0>(tracks);
            std::map<long, long> id_to_track_id = std::get<1>(tracks);

            std::map<long, rasberry_perception::Detection> id_to_last_det;
            for(auto & last_det : this->last_detections_msg_.objects) {
                id_to_last_det[last_det.id] = last_det;
            }

            if (!detections.empty()) {
                ros::Time now =  ros::Time::now();
                // Publish the markers that are currently visible
                rasberry_perception::Detections non_occluded_results = this->last_detections_msg_;
                non_occluded_results.camera_frame.header.frame_id = this->target_frame;
                non_occluded_results.camera_frame.header.stamp = now;
                non_occluded_results.camera_info.header.frame_id = this->target_frame;
                rasberry_perception::TaggedPoseStampedArray tagged_poses;
                tagged_poses.header = non_occluded_results.camera_info.header;
                tagged_poses.header.stamp = now;

                // Publish a message with only the currently visible detections
                for(auto & detection : non_occluded_results.objects) {
                    // Detection has been associated to a track otherwise leave as track id -1
                    detection.track_id = id_to_track_id.find(detection.id) != id_to_track_id.end() ? id_to_track_id[detection.id] : -1;
                    detection.pose_frame_id = this->target_frame;
                    this->pub_detection_results.publish(detection); // rasberry_perception/Detection
                }

                if(this->pub_detection_results_array.getNumSubscribers())
                    this->pub_detection_results_array.publish(non_occluded_results); // rasberry_perception/Detections

                if(pub_detection_markers.getNumSubscribers()) {
                    visualization_msgs::MarkerArray detection_markers = createVisualisationMarkers(non_occluded_results);
                    this->pub_detection_markers.publish(detection_markers); // visualization_msgs::MarkerArray
                }

                rasberry_perception::Detections results = non_occluded_results;
                results.objects.clear();

                // Also publish the tracker results array
                for(auto & detection : detections) {
                    // TODO (raymond): Use vel and var as well as pose
                    rasberry_perception::Detection det = detection.second[0]; // 0=pose, 1=vel, 2=var

                    // If this track is still in the most recent history publish the information
                    if(id_to_last_det.find(det.id) != id_to_last_det.end())
                        if(id_to_track_id[det.id] == det.track_id) {
                            det.roi = id_to_last_det[det.id].roi;
                            det.seg_roi = id_to_last_det[det.id].seg_roi;
                            det.confidence = id_to_last_det[det.id].confidence;
                        }

                    det.pose_frame_id = this->target_frame;
                    results.objects.push_back(det);
                    this->pub_results.publish(det); // rasberry_perception/Detection

                    rasberry_perception::TaggedPose tp; tp.tag = det.class_name; tp.pose = det.pose;
                    tagged_poses.poses.push_back(tp);
                    this->pub_pose_results.publish(tp); // rasberry_perception/TaggedPose
                }

                if (pub_results.getNumSubscribers() || pub_results_array.getNumSubscribers()) {
                    this->pub_results_array.publish(results); // rasberry_perception/Detections
                    this->pub_pose_results_array.publish(tagged_poses); // rasberry_perception/TaggedPoseStampedArray
                }

                if(pub_markers.getNumSubscribers()) {
                    visualization_msgs::MarkerArray markers = createVisualisationMarkers(results);
                    this->pub_markers.publish(markers); // visualization_msgs::MarkerArray
                }
            }

            fps.sleep();
        }
        catch (std::exception &e) {
            ROS_INFO_STREAM("Exception: " << e.what());
            fps.sleep();
        }
        catch (Bayesian_filter::Numeric_exception &e) {
            ROS_INFO_STREAM("Exception: " << e.what());
            fps.sleep();
        }
    }
}

visualization_msgs::MarkerArray Tracking::createVisualisationMarkers(const rasberry_perception::Detections &detections) {
    visualization_msgs::MarkerArray marker_array;

    bool real_size = false;
    geometry_msgs::Vector3 scale;
    scale.x = 0.025;
    scale.y = 0.025;
    scale.z = 0.025;

    std_msgs::ColorRGBA red;
    red.a = 1.0;
    red.r = 215.0F / 255.0F;
    red.g = 10.0F / 255.0F;
    red.b = 100.0F / 255.0F;

    std_msgs::ColorRGBA green;
    green.a = 1.0;
    green.r = 0.0F / 255.0F;
    green.g = 200.0F / 255.0F;
    green.b = 0.0F / 255.0F;

    for (const auto & object : detections.objects) {
        if(object.track_id == -1) {
            continue;
        }

        ros::Time now = ros::Time::now();
        std::string ns = "rasberry_perception/tracking";

        visualization_msgs::Marker marker;
        marker.header.frame_id = object.pose_frame_id;
        marker.header.stamp = now;
        marker.header.seq = ++marker_seq;
        marker.ns = ns;
        marker.id = object.track_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose = object.pose;

        // TODO (raymond): This feature needs work
        if(real_size) {
            scale.x = std::abs(((object.roi.x2 - object.roi.x1) - detections.camera_info.P.elems[2]) * object.pose.position.z / detections.camera_info.P.elems[0]);
            scale.y = std::abs(((object.roi.y2 - object.roi.y1) - detections.camera_info.P.elems[6]) * object.pose.position.z / detections.camera_info.P.elems[5]);
            scale.z = scale.x;
        }

        marker.scale = scale;
        marker.color = red;
        marker.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = object.pose_frame_id;
        text_marker.header.stamp = now;
        text_marker.header.seq = marker.header.seq;
        text_marker.ns = ns;
        text_marker.id = -object.track_id - 100;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.text = num_to_str<long>(object.track_id);
        text_marker.action = visualization_msgs::Marker::MODIFY;
        text_marker.pose = object.pose;
        text_marker.pose.position.y -= scale.y * 2;
        text_marker.scale.z = scale.z;
        text_marker.color = green;
        text_marker.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(text_marker);
    }

    return marker_array;
}

void Tracking::detectorCallback(const rasberry_perception::Detections::ConstPtr &results,
                                const std::string &detector) {
    if(results->objects.empty()) {
        return;
    }

    rasberry_perception::Detections detections(*results);

    // Set last header to the most recent detection header
    this->last_header_ = detections.camera_info.header;
    this->last_detections_msg_ = detections;

    std::vector<geometry_msgs::Point> position;
    std::vector<std::string> tags;
    std::vector<int> detection_ids;

    for (auto & object : detections.objects) {
        geometry_msgs::Pose pt = object.pose;
        if(object.pose_frame_id.empty()) {
            ROS_WARN("Detection %ld pose_frame_id is empty. Skipping object.", object.id);
            continue;
        }

        //Create stamped pose for tf
        geometry_msgs::PoseStamped pose_in_detection_coords;
        geometry_msgs::PoseStamped pose_in_target_coords;
        pose_in_detection_coords.header = detections.camera_info.header;
        pose_in_detection_coords.pose = pt;

        try {
            // Transform into given target frame.
            std::string current_frame = object.pose_frame_id;
            if (strcmp(target_frame.c_str(), current_frame.c_str()) != 0) {
                ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
                listener->waitForTransform(current_frame, target_frame, ros::Time(0),ros::Duration(3.0));
                listener->transformPose(target_frame, ros::Time(0), pose_in_detection_coords, current_frame, pose_in_target_coords);
            } else {
                pose_in_target_coords = pose_in_detection_coords;
            }
        } catch (const tf::TransformException &ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            return;
        }

        position.push_back(pose_in_target_coords.pose.position);
        detection_ids.push_back(object.id);

        if (_use_tags[detector]) {
            tags.push_back(object.class_name);
        }
    }

    if (!position.empty()) {
        if(ekf != nullptr) {
            ekf->addObservation(detector, position, this->last_header_.stamp.toSec(), tags, detection_ids);
        } else if (ukf != nullptr) {
            ukf->addObservation(detector, position, this->last_header_.stamp.toSec(), tags, detection_ids);
        } else if (pf != nullptr) {
            pf->addObservation(detector, position, this->last_header_.stamp.toSec(), tags, detection_ids);
        }
    }
}

void Tracking::detectorCallbackPoseArray(const geometry_msgs::PoseArray::ConstPtr &results, const std::string &detector) {
    if (results->poses.empty()) {
        return;
    }

    if (_use_tags[detector]) {
        ROS_FATAL_STREAM("'*-LABELLED' matching_algorithm used but PoseArray topic received for '" << detector << "'");
        throw reset_exception();
    }

    // Shiv between TaggedPoseStampedArray and PoseArray (copy results to TaggedPoseStampedArray with empty tags)
    rasberry_perception::TaggedPoseStampedArray::Ptr tagged_res(new rasberry_perception::TaggedPoseStampedArray());
    for (auto &it : results->poses) {
        auto temp_pose = rasberry_perception::TaggedPose();
        temp_pose.pose = it;
        temp_pose.tag = "";
        tagged_res->poses.push_back(temp_pose);
    }
    tagged_res->header = results->header;

    rasberry_perception::TaggedPoseStampedArray::ConstPtr const_tagged_res(tagged_res);
    this->detectorCallbackTaggedPoseStampedArray(const_tagged_res, detector);
}

void Tracking::detectorCallbackTaggedPoseStampedArray(const rasberry_perception::TaggedPoseStampedArray::ConstPtr &results,
                                               const std::string &detector) {
    if (results->poses.empty()) {
        return;
    }

    std::vector<geometry_msgs::Point> position;
    std::vector<std::string> tags;
    std::vector<int> detection_ids; // Fill with seq because tagged poses do not have IDS for each detection

    for (int i = 0; i < results->poses.size(); i++) {
        rasberry_perception::TaggedPose pt = results->poses[i];
        //Create stamped pose for tf
        geometry_msgs::PoseStamped pose_in_cam_coords;
        geometry_msgs::PoseStamped pose_in_target_coords;
        pose_in_cam_coords.header = results->header;
        pose_in_cam_coords.pose = pt.pose;

        // Set last header to the most recent detection header
        this->last_header_ = results->header;

        //Transform
        try {
            // Transform into given target frame. Default /base
            if (strcmp(target_frame.c_str(), pose_in_cam_coords.header.frame_id.c_str()) != 0) {
                ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
                listener->waitForTransform(pose_in_cam_coords.header.frame_id, target_frame, ros::Time(0),
                                           ros::Duration(3.0));
                listener->transformPose(target_frame, ros::Time(0), pose_in_cam_coords, pose_in_cam_coords.header.frame_id,
                                        pose_in_target_coords);
            } else {
                pose_in_target_coords = pose_in_cam_coords;
            }
        } catch (const tf::TransformException &ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            return;
        }

        position.push_back(pose_in_target_coords.pose.position);
        detection_ids.push_back(results->header.seq);
        if (_use_tags[detector]) {
            tags.push_back(pt.tag);
        }
    }

    if (!position.empty()) {
        if (ekf == nullptr) {
            if (ukf == nullptr) {
                pf->addObservation(detector, position, results->header.stamp.toSec(), tags, detection_ids);
            } else {
                ukf->addObservation(detector, position, results->header.stamp.toSec(), tags, detection_ids);
            }
        } else {
            ekf->addObservation(detector, position, results->header.stamp.toSec(), tags, detection_ids);
        }
    }
}


void Tracking::resetCallback(const std_msgs::String::ConstPtr &reset_reason) {
    ROS_INFO_STREAM("Resetting detector due to: " << reset_reason->data);
    throw reset_exception();
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rasberry_tracking", ros::init_options::AnonymousName);

    while (ros::ok()) {
        // Run the tracker, if a reset is requested reinitialise the node
        try {
            auto *t = new Tracking();
        } catch (const reset_exception &e) {
            ROS_FATAL_STREAM(e.what());
            ROS_FATAL_STREAM("Resetting in 3 seconds.");
            ros::Duration(3).sleep();
            continue;
        }
        break;
    }

    return 0;
}

