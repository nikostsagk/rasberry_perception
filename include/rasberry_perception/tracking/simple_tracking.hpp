/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto                                 *
 *   nbellotto@lincoln.ac.uk                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef SIMPLE_TRACKING_H
#define SIMPLE_TRACKING_H

#include <ros/ros.h>
#include <ros/time.h>
#include "rasberry_perception/TaggedPose.h"
#include <bayes_tracking/multitracker.h>
#include <bayes_tracking/models.h>
#include <bayes_tracking/ekfilter.h>
#include <bayes_tracking/ukfilter.h>
#include <bayes_tracking/pfilter.h>
#include <cstdio>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/optional.hpp>
#include <math.h>

using namespace std;
using namespace MTRK;
using namespace Models;

// rule to detect lost track
template<class FilterType>
bool MTRK::isLost(const FilterType *filter, double stdLimit) {
    // track lost if var(x)+var(y)+var(z) > stdLimit^2
    bool is_lost = filter->X(0, 0) + filter->X(2, 2) + filter->X(4, 4) > sqr(stdLimit);

//    ROS_INFO("SUM(var_x: %f, var_y: %f, var_z: %f = %f) > stdLim: %f = %i",filter->X(0,0), filter->X(2,2), filter->X(4,4),
//             filter->X(0, 0) + filter->X(2, 2) + filter->X(4, 4), sqr(stdLimit), is_lost);

    return is_lost;
}

// rule to create new track
template<class FilterType>
bool MTRK::initialize(FilterType *&filter, sequence_t &obsvSeq, observ_model_t om_flag) {
    assert(!obsvSeq.empty());

    if (om_flag == CARTESIAN) {
        double dt = obsvSeq.back().time - obsvSeq.front().time;
        if (dt == 0) return false;
        //assert(dt); // dt must not be null

        FM::Vec v((obsvSeq.back().vec - obsvSeq.front().vec) / dt);

//        int cmp_v = 2;
//        if(v[0] < -cmp_v || v[0] > cmp_v || v[1] < -cmp_v || v[1] > cmp_v || v[2] < -cmp_v || v[2] > cmp_v) {
//            return false;
//        }
//
//        ROS_INFO_STREAM("v: " << v);

        FM::Vec x(6);
        FM::SymMatrix X(6, 6);

        x[0] = obsvSeq.back().vec[0];
        x[1] = v[0];
        x[2] = obsvSeq.back().vec[1];
        x[3] = v[1];
        x[4] = obsvSeq.back().vec[2];
        x[5] = v[2];
        X.clear();
        X(0, 0) = sqr(0.01);
        X(1, 1) = sqr(0.10);
        X(2, 2) = sqr(0.01);
        X(3, 3) = sqr(0.10);
        X(4, 4) = sqr(0.01);
        X(5, 5) = sqr(0.10);
        filter = new FilterType(6);
        filter->init(x, X);
    }

    if (om_flag == POLAR) {
        double dt = obsvSeq.back().time - obsvSeq.front().time;
        if (dt == 0) return false;
        //assert(dt); // dt must not be null
        double x2 = obsvSeq.back().vec[1] * cos(obsvSeq.back().vec[0]);
        double y2 = obsvSeq.back().vec[1] * sin(obsvSeq.back().vec[0]);
        double x1 = obsvSeq.front().vec[1] * cos(obsvSeq.front().vec[0]);
        double y1 = obsvSeq.front().vec[1] * sin(obsvSeq.front().vec[0]);

        FM::Vec x(4);
        FM::SymMatrix X(4, 4);

        x[0] = x2;
        x[1] = (x2 - x1) / dt;
        x[2] = y2;
        x[3] = (y2 - y1) / dt;
        X.clear();
        X(0, 0) = sqr(0.5);
        X(1, 1) = sqr(1.5);
        X(2, 2) = sqr(0.5);
        X(3, 3) = sqr(1.5);

        filter = new FilterType(4);
        filter->init(x, X);
    }

    return true;
}

template<typename FilterType>
class SimpleTracking {
public:
    explicit SimpleTracking(double sLimit = 1.0, bool prune_named_tracks = false) {
        time = getTime();
        observation = new FM::Vec(3);
        stdLimit = sLimit;
        prune_named = prune_named_tracks;
    }

    void createConstantVelocityModel(double vel_noise_x, double vel_noise_y, double vel_noise_z) {
        cvm = new CVModel3D(vel_noise_x, vel_noise_y, vel_noise_z);
    }

    void addDetectorModel(std::string name, association_t alg, observ_model_t om_flag, double pos_noise_x,
                          double pos_noise_y, double pos_noise_z = 0, unsigned int seqSize = 5, double seqTime = 0.2) {
        ROS_INFO("Adding detector model for: %s.", name.c_str());
        detector_model det;
        det.om_flag = om_flag;
        det.alg = alg;
        if (om_flag == CARTESIAN)
            det.ctm = new CartesianModel3D(pos_noise_x, pos_noise_y, pos_noise_z);
        if (om_flag == POLAR)
            det.plm = new PolarModel(pos_noise_x, pos_noise_y);
        det.seqSize = seqSize;
        det.seqTime = seqTime;
        detectors[name] = det;
    }

    std::map<long, std::vector<rasberry_perception::Detection>> track(double *track_time, std::map<long, std::string> &tags) {
        boost::mutex::scoped_lock lock(mutex);
        std::map<long, std::vector<rasberry_perception::Detection>>  result; // bbox id to detection info (including track)
        dt = getTime() - time;
        time += dt;
        if (track_time) *track_time = time;

        // This function is only for the prediction step of the tracker. Thus, the update step is not performed here
        // Since the update step is related to observations (from detectors), it is performed in addObservation function, when a new observation comes
        // for(typename std::map<std::string, detector_model>::const_iterator it = detectors.begin();
        //    it != detectors.end();
        //    ++it) {
        //    // prediction
        //    cvm->update(dt);
        //    mtrk.template predict<CVModel>(*cvm);

        //    // process observations (if available) and update tracks
        //    mtrk.process(*(it->second.ctm), it->second.alg);
        //}
        // prediction
        cvm->update(dt);
        mtrk.template predict<CVModel3D>(*cvm);
        //detector_model dummy_det;
        //mtrk.process(*(dummy_det.ctm));
        mtrk.pruneTracks(stdLimit);
        if (prune_named) {
            mtrk.pruneNamedTracks();
        }

        for (int i = 0; i < mtrk.size(); i++) {
            double theta = atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]);
        }

        for (int i = 0; i < mtrk.size(); i++) {
            double theta = atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]);
            ROS_DEBUG("trk_%ld: Position: (%f, %f), Orientation: %f, Std Deviation: %f, %f",
                      mtrk[i].id,
                      mtrk[i].filter->x[0], mtrk[i].filter->x[2], //x, y
                      theta, //orientation
                      sqrt(mtrk[i].filter->X(0, 0)), sqrt(mtrk[i].filter->X(2, 2))//std dev
            );
            rasberry_perception::Detection pose, vel, var; // position, velocity, variance
            // If no tag then the label is the track ID
            std::string label = mtrk[i].tag.empty() ? std::to_string(mtrk[i].id) : mtrk[i].tag;
            pose.pose.position.x = mtrk[i].filter->x[0];
            pose.pose.position.y = mtrk[i].filter->x[2];
            pose.pose.position.z = mtrk[i].filter->x[4];
            pose.pose.orientation.z = sin(theta / 2);
            pose.pose.orientation.w = cos(theta / 2);
            pose.id = -1; // TODO: Replace with original detection ID
            pose.class_name = mtrk[i].tag;
            pose.track_id = mtrk[i].id;
            result[mtrk[i].id].push_back(pose);

            vel.pose.position.x = mtrk[i].filter->x[1];
            vel.pose.position.y = mtrk[i].filter->x[3];
            vel.pose.position.z = mtrk[i].filter->x[5];
            vel.id = -1; // TODO: Replace with original detection ID
            vel.class_name = mtrk[i].tag;
            vel.track_id = mtrk[i].id;
            result[mtrk[i].id].push_back(vel);

            var.pose.position.x = mtrk[i].filter->X(0, 0);
            var.pose.position.y = mtrk[i].filter->X(2, 2);
            var.pose.position.z = mtrk[i].filter->X(4, 4);
            var.id = -1; // TODO: Replace with original detection ID
            var.class_name = mtrk[i].tag;
            var.track_id = mtrk[i].id;

            tags[mtrk[i].id] = mtrk[i].tag;

            // TODO: Replace track_id with detection ID
            result[mtrk[i].id].push_back(var);
        }

        return result;
    }

    void addObservation(std::string detector_name, const std::vector<geometry_msgs::Point>& obsv, double obsv_time,
                        std::vector<std::string> tags) {
        boost::mutex::scoped_lock lock(mutex);
        ROS_DEBUG("Adding new observations for detector: %s", detector_name.c_str());
        // add last observation/s to tracker
        detector_model det;
        try {
            det = detectors.at(detector_name);
        } catch (std::out_of_range &exc) {
            ROS_ERROR("Detector %s was not registered!", detector_name.c_str());
            return;
        }

        dt = getTime() - time;
        time += dt;

        // prediction
        cvm->update(dt);
        mtrk.template predict<CVModel3D>(*cvm);

        // mtrk.process(*(det.ctm), det.alg);

        int count = 0;
        for (auto & li : obsv) {
            if (det.om_flag == CARTESIAN) {
                (*observation)[0] = li.x;
                (*observation)[1] = li.y;
                (*observation)[2] = li.z;
            }
            if (det.om_flag == POLAR) {
                (*observation)[0] = atan2(li.y, li.x); // bearing
                (*observation)[1] = sqrt(pow(li.x, 2) + pow(li.y, 2)); // range
            }
            if (!tags.empty() && count < tags.size())
                mtrk.addObservation(*observation, obsv_time, tags[count]);
            else
                mtrk.addObservation(*observation, obsv_time);
            count++;
        }

        if (det.om_flag == CARTESIAN) {
            mtrk.process(*(det.ctm), det.alg, det.seqSize, det.seqTime, stdLimit, det.om_flag);
        }

        if (det.om_flag == POLAR) {
            //det.plm->update(robot_pose.position.x, robot_pose.position.y, robot_pose.orientation.w);
            det.plm->update(0, 0, 0);
            mtrk.process(*(det.plm), det.alg, det.seqSize, det.seqTime, stdLimit, det.om_flag);
        }
        count++;
    }

private:

    FM::Vec *observation;           // observation [x, y, z] (z for cartesian)
    double dt{}, time;
    boost::mutex mutex;
    CVModel3D *cvm{};                   // CV model
    MultiTracker<FilterType, 6> mtrk; // state [x, v_x, y, v_y, z, v_z]
    double stdLimit;
    bool prune_named;                  // upper limit for the variance of estimation position

    typedef struct {
        CartesianModel3D *ctm;        // Cartesian observation model
        PolarModel *plm;            // Polar observation model
        observ_model_t om_flag;     // Observation model flag
        association_t alg;          // Data association algorithm
        unsigned int seqSize;       // Minimum number of observations for new track creation
        double seqTime;             // Minimum interval between observations for new track creation
    } detector_model;
    std::map<std::string, detector_model> detectors;

    double getTime() {
        return ros::Time::now().toSec();
    }
};

#endif //SIMPLE_TRACKING_H