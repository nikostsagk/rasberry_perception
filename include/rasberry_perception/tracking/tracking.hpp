/**
 * ROS Node used to interact with baysian tracker
 * based on bayes_people_tracker <https://github.com/strands-project/strands_perception_people/>
 * by Christian Dondrup <cdondrup@lincoln.ac.uk>
 * 
 *  
 * @author Peter Lightbody
 * @Version 0.3
 * @Organization University of Lincoln
 * @Status Development
 * @Copyright The MIT License (MIT)
 * @Date 22 May 2015
 * 
 */

#ifndef BTRACKING_H
#define BTRACKING_H

#include "rasberry_perception/TaggedPose.h"
#include "rasberry_perception/TaggedPoseStampedArray.h"
#include "rasberry_perception/Detection.h"
#include "rasberry_perception/Detections.h"

#include "rasberry_perception/tracking/asso_exception.hpp"
#include "rasberry_perception/tracking/simple_tracking.hpp"
#include <bayes_tracking/BayesFilter/bayesFlt.hpp>
#include <boost/bind.hpp>
#include <std_msgs/String.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>
#include <XmlRpcValue.h>


class Tracking {
public:
    Tracking();

private:
    template<typename T>
    std::string num_to_str(T num) {
        std::stringstream ss;
        ss << num;
        return ss.str();
    }

    int parseParams(ros::NodeHandle n);
    void trackingThread();

    visualization_msgs::MarkerArray createVisualisationMarkers(const rasberry_perception::Detections& poses);

    void detectorCallback(const rasberry_perception::Detections::ConstPtr &results, const std::string& detector);
    void detectorCallbackPoseArray(const geometry_msgs::PoseArray::ConstPtr &results, const std::string& detector);
    void detectorCallbackTaggedPoseStampedArray(const rasberry_perception::TaggedPoseStampedArray::ConstPtr &results, const std::string& detector);
    static void resetCallback(const std_msgs::String::ConstPtr & reset_reason);

    // MEMBER VARIABLES
    tf::TransformListener* listener;
    ros::Publisher pub_results, pub_results_array, pub_detection_results, pub_detection_results_array, pub_pose_results,
                   pub_pose_results_array, pub_markers, pub_detection_markers;
    rasberry_perception::Detections last_detections_msg_;
    std_msgs::Header last_header_;
    std::string target_frame, startup_time_str;
    double tracker_frequency{};
    double startup_time;

    unsigned long detect_seq;
    unsigned long marker_seq;
    SimpleTracking<PFilter> *pf{}; //= NULL;
    SimpleTracking<EKFilter> *ekf{}; //= NULL;
    SimpleTracking<UKFilter> *ukf{}; //= NULL;
    std::map<std::string, bool> _use_tags; // <detector name, use tags>
    std::vector<ros::Subscriber> subs; //A store to keep the subscriber objects alive
};

#endif // BTRACKING_H
