/*
 * Copyright (C)2018  ICub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <cmath>
#include <random>
#include <algorithm>
#include "amclLocalizer.h"

using namespace yarp::os;
using namespace amcl;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

#define  LOWLEVEL_DEBUG


static double normalize(double z)
{
    return atan2(sin(z), cos(z));
}

static double angle_diff(double a, double b)
{
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2 * M_PI - fabs(d1);
    if (d1 > 0)
        d2 *= -1.0;
    if (fabs(d1) < fabs(d2))
        return(d1);
    else
        return(d2);
}

void amclLocalizerRPCHandler::setInterface(amclLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool amclLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   amclLocalizer::getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status)
{
    status = yarp::dev::LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   amclLocalizer::getEstimatedPoses(std::vector<yarp::dev::Map2DLocation>& poses)
{
    poses.clear();
    thread->getPoses(poses);
    return true;
}

bool   amclLocalizer::getCurrentPosition(yarp::dev::Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return true;
}

bool   amclLocalizer::setInitialPose(yarp::dev::Map2DLocation& loc)
{
    thread->initializeLocalization(loc);
    return true;
}

//////////////////////////

amclLocalizerThread::amclLocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_handler_odom = nullptr;
    m_handler_pf = nullptr;
    m_handler_laser = nullptr;
    m_initial_pose_hyp = nullptr;
    m_amcl_map = nullptr;
    m_iMap = nullptr;
    m_iLaser = nullptr;

    m_last_odometry_data_received = -1;
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = nan("");
    m_localization_data.y = nan("");
    m_localization_data.theta = nan("");
}

void amclLocalizerThread::updateFilter()
{
    int laser_index = 0; //@@@always use the first and unique (for now) laser

    pf_vector_t delta = pf_vector_zero();
    pf_vector_t pose;
    pose.v[0] = m_odometry_data.x;
    pose.v[1] = m_odometry_data.y;
    pose.v[2] = m_odometry_data.theta*DEG2RAD; //@@@@ CHECK THIS!!!

    if (m_pf_init)
    {
#ifdef LOWLEVEL_DEBUG
        yDebug() << "m_pf_init=true, it means pf is initialized";
#endif
        // Compute change in pose
        delta.v[0] = pose.v[0] - m_pf_odom_pose.v[0];
        delta.v[1] = pose.v[1] - m_pf_odom_pose.v[1];
        delta.v[2] = angle_diff(pose.v[2], m_pf_odom_pose.v[2]);

        // See if we should update the filter
        bool update = fabs(delta.v[0]) > m_config.m_d_thresh ||
                      fabs(delta.v[1]) > m_config.m_d_thresh ||
                      fabs(delta.v[2]) > m_config.m_a_thresh;

#ifdef LOWLEVEL_DEBUG
        if (update)
        {   yDebug() << "Update requested by thresholds";   }
        else
        {   yDebug() << "No threshold update requested";   }
        if(m_force_update)
        {
            yDebug() << "Force Update requested";
        }
#endif

        update = update || m_force_update;
        m_force_update = false;

        // Set the laser update flags
        if (update)
        {
            for (unsigned int i = 0; i < m_lasers_update.size(); i++)
            {
                m_lasers_update[i] = true;
            }
#ifdef LOWLEVEL_DEBUG
            yDebug() << "1. Laser updated = true";
#endif
        }
        else
        {
#ifdef LOWLEVEL_DEBUG
            yDebug() << "1. Laser updated unrequested";
#endif
        }
    }

    bool force_publication = false;
    //first run, filter initialization
    if (!m_pf_init)
    {
#ifdef LOWLEVEL_DEBUG
        yDebug() << "m_pf_init=false, it means pf is NOT YET initialized";
#endif
        // Pose at last filter update
        m_pf_odom_pose = pose;
        // Filter is now initialized
        m_pf_init = true;
        // Should update sensor data
        for (unsigned int i = 0; i < m_lasers_update.size(); i++)
        {
            m_lasers_update[i] = true;
        }
        force_publication = true;
        m_resample_count = 0;
    }
    // If the robot has moved, update the filter
    else if (m_pf_init && m_lasers_update[laser_index])
    {
#ifdef LOWLEVEL_DEBUG
        yDebug() << "m_pf_init=true, m_lasers_update=true. update odometry";
#endif
        //printf("pose\n");
        //pf_vector_fprintf(pose, stdout, "%.3f");

        AMCLOdomData odata;
        odata.pose = pose;
        // HACK
        // Modify the delta in the action data so the filter gets
        // updated correctly
        odata.delta = delta;

        // Use the action data to update the filter
        m_handler_odom->UpdateAction(m_handler_pf, (AMCLSensorData*)&odata);

        // Pose at last filter update
        //this->pf_odom_pose = pose;
    }

    bool resampled = false;
    // If the robot has moved, update the filter
    if (m_lasers_update[laser_index])
    {
#ifdef LOWLEVEL_DEBUG
        yDebug() << "m_lasers_update=true, update laser data";
#endif
        AMCLLaserData ldata;
        ldata.sensor = m_lasers[laser_index];
        ldata.range_count = m_laser_measurement_data.size(); //@@@ 360? get this form the laser
        double angle_min = m_min_laser_angle * DEG2RAD; ///@@@ get this from the laser, THIS needs to be expressed in the base frame, in RADIAS
        double angle_increment = 1.0*DEG2RAD; //@@@ THIS needs to expressed in the base frame, in RADIANS

        // wrapping angle to [-pi .. pi]
        angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI; //@@@CHEKC THIS
#ifdef  LOWLEVEL_DEBUG 
        yDebug("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);
#endif

        // Apply range min/max thresholds, if the user supplied them
        if (m_config.m_laser_max_range > 0.0)
        {
            double tmp = m_max_laser_distance;
            ldata.range_max = std::min(tmp, m_config.m_laser_max_range);
        }
        else
        {
            ldata.range_max = m_max_laser_distance;
        }
        double range_min;
        if (m_config.m_laser_min_range > 0.0)
        {
            double tmp = m_min_laser_distance;
            range_min = std::max(tmp, m_config.m_laser_min_range);
        }
        else
        {
            range_min = m_min_laser_distance;
        }
        // The AMCLLaserData destructor will free this memory
        ldata.ranges = new double[ldata.range_count][2];
        yAssert(ldata.ranges);
        for (int i = 0; i<ldata.range_count; i++)
        {
            // amcl doesn't (yet) have a concept of min range.  So we'll map short readings to max range.
            double rho = 0;
            double theta = 0;
            m_laser_measurement_data[i].get_polar(rho,theta); //@@@@ check carefully, i and theta
            if (rho <= range_min)
            {
                ldata.ranges[i][0] = ldata.range_max;
            }
            else
            {
                ldata.ranges[i][0] = rho;
            }
            // Compute bearing
            ldata.ranges[i][1] = angle_min + (i * angle_increment);
        }

        m_lasers[laser_index]->UpdateSensor(m_handler_pf, (AMCLSensorData*)&ldata);

        m_lasers_update[laser_index] = false;

        m_pf_odom_pose = pose;

        // Resample the particles
        if (!(++m_resample_count % m_resample_interval))
        {
            pf_update_resample(m_handler_pf);
            yDebug("Resampled by time (count %d / %d)", m_resample_count, m_resample_interval);
            resampled = true;
        }

        pf_sample_set_t* set = m_handler_pf->sets + m_handler_pf->current_set;
#ifdef LOWLEVEL_DEBUG
        yDebug("Num samples: %d\n", set->sample_count);
#endif
        // Publish the resulting cloud
        // TODO: set maximum rate for publishing
        if (!m_force_update)
        {
            //mutex protected vs amclLocalizerThread::getPoses()
            m_particle_poses_mutex.lock();
            m_particle_poses.clear();
            for (int i = 0; i < set->sample_count; i++)
            {
                yarp::dev::Map2DLocation ppose;
                ppose.x = set->samples[i].pose.v[0];
                ppose.y = set->samples[i].pose.v[1];
                ppose.theta = set->samples[i].pose.v[2]*RAD2DEG; //@@@@@@@CHECKME
                m_particle_poses.push_back(ppose);
            }
            m_particle_poses_mutex.unlock();
        }
    }

    if (resampled || force_publication)
    {
        // Read out the current hypotheses
        double max_weight = 0.0;
        int max_weight_hyp = -1;
        std::vector<amcl_hyp_t> hyps;
        hyps.resize(m_handler_pf->sets[m_handler_pf->current_set].cluster_count);
        for (int hyp_count = 0;
            hyp_count < m_handler_pf->sets[m_handler_pf->current_set].cluster_count; hyp_count++)
        {
            double weight;
            pf_vector_t pose_mean;
            pf_matrix_t pose_cov;
            if (!pf_get_cluster_stats(m_handler_pf, hyp_count, &weight, &pose_mean, &pose_cov))
            {
                yError("Couldn't get stats on cluster %d", hyp_count);
                break;
            }

            hyps[hyp_count].weight = weight;
            hyps[hyp_count].pf_pose_mean = pose_mean;
            hyps[hyp_count].pf_pose_cov = pose_cov;

            if (hyps[hyp_count].weight > max_weight)
            {
                max_weight = hyps[hyp_count].weight;
                max_weight_hyp = hyp_count;
            }
        }

        if (max_weight > 0.0)
        {
            yDebug("Max weight pose: %.3f %.3f %.3f (%.3f)",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2],
                hyps[max_weight_hyp].pf_pose_mean.v[2]*RAD2DEG);

            m_localization_data_mutex.lock();
            m_localization_data.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
            m_localization_data.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
            m_localization_data.theta = hyps[max_weight_hyp].pf_pose_mean.v[2] * RAD2DEG;
            m_localization_data_mutex.unlock();
        }

    }
}

bool amclLocalizerThread::getPoses(std::vector<yarp::dev::Map2DLocation>& poses)
{
    LockGuard lock(m_particle_poses_mutex);
    poses = m_particle_poses;
    return true;
}

void amclLocalizerThread::run()
{
#ifdef LOWLEVEL_DEBUG
    yDebug();
#endif // LOWLEVEL_DEBUG

    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    LockGuard lock(m_mutex);

    //read odometry data
    yarp::sig::Vector *loc = m_port_odometry_input.read(false);
    if (loc)
    {
        m_last_odometry_data_received = yarp::os::Time::now();
        m_odometry_data.x = loc->data()[0];
        m_odometry_data.y = loc->data()[1];
        m_odometry_data.theta = loc->data()[2];
    }
    if (current_time - m_last_odometry_data_received > 0.1)
    {
        yWarning() << "No localization data received for more than 0.1s!";
    }

    //read laser data
    bool las_ok = m_iLaser->getLaserMeasurement(m_laser_measurement_data);
    if (las_ok)
    {
        m_laser_measurement_timestamp = yarp::os::Time::now();
        pf_vector_t pose_v;
        //@@@@set here the pose of the laser respect to base_frame_id
        pose_v.v[0] = 0;
        pose_v.v[1] = 0;
        pose_v.v[2] = 0;
        m_lasers[0]->SetLaserPose(pose_v);
    }

    //process data
    updateFilter();
}

bool amclLocalizerThread::initializeLocalization(yarp::dev::Map2DLocation& loc)
{
    LockGuard lock(m_mutex);
    m_localization_data.map_id = loc.map_id;
    m_localization_data.x = loc.x;
    m_localization_data.y = loc.y;
    m_localization_data.theta = loc.theta;

    // Re-initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = loc.x;
    pf_init_pose_mean.v[1] = loc.y;
    pf_init_pose_mean.v[2] = loc.theta * DEG2RAD; //@@@@ check me
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();

    pf_init_pose_cov.m[0][0] = m_initial_covariance_msg[0][0];
    pf_init_pose_cov.m[0][1] = m_initial_covariance_msg[0][1];
    pf_init_pose_cov.m[1][0] = m_initial_covariance_msg[1][0];
    pf_init_pose_cov.m[1][1] = m_initial_covariance_msg[1][1];
    pf_init_pose_cov.m[2][2] = m_initial_covariance_msg[2][2];

    // Copy in the covariance, converting from 6-D to 3-D
    /*
    // @@@@@@ check me
    for (int i = 0; i<2; i++)
    {
        for (int j = 0; j<2; j++)
        {
            pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
        }
    }
    pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];
    */

    if (m_initial_pose_hyp)
    {
        delete m_initial_pose_hyp;
        m_initial_pose_hyp = nullptr;
    }
    m_initial_pose_hyp = new amcl_hyp_t();
    m_initial_pose_hyp->pf_pose_mean = pf_init_pose_mean;
    m_initial_pose_hyp->pf_pose_cov = pf_init_pose_cov;

    applyInitialPose();
    return true;
}

bool amclLocalizerThread::getCurrentLoc(yarp::dev::Map2DLocation& loc)
{
    LockGuard lock (m_localization_data_mutex);
    loc = m_localization_data;
    return true;
}

bool amclLocalizerThread::threadInit()
{
    //configuration file checking
    Bottle general_group = m_cfg.findGroup("GENERAL");
    if (general_group.isNull())
    {
        yError() << "Missing GENERAL group!";
        return false;
    }

    Bottle initial_group = m_cfg.findGroup("INITIAL_POS");
    if (initial_group.isNull())
    {
        yError() << "Missing INITIAL_POS group!";
        return false;
    }

    Bottle amcl_group = m_cfg.findGroup("AMCL");
    if (amcl_group.isNull())
    {
        yError() << "Missing AMCL group!";
        return false;
    }

    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yError() << "Missing LOCALIZATION group!";
        return false;
    }

    Bottle tf_group = m_cfg.findGroup("TF");
    if (tf_group.isNull())
    {
        yError() << "Missing TF group!";
        return false;
    }

    Bottle odometry_group = m_cfg.findGroup("ODOMETRY");
    if (odometry_group.isNull())
    {
        yError() << "Missing ODOMETRY group!";
        return false;
    }
    Bottle laser_group = m_cfg.findGroup("LASER");
    if (laser_group.isNull())
    {
        yError() << "Missing LASER group!";
        return false;
    }

    //general group
    m_local_name = "amclLocalizer";
    if (general_group.check("local_name")) { m_local_name = general_group.find("local_name").asString(); }

    //laser group
    if (laser_group.check("laser_broadcast_port") == false)
    {
        yError() << "Missing `laser_broadcast_port` in [LASER] group";
        return false;
    }
    m_laser_remote_port = laser_group.find("laser_broadcast_port").asString();

    //odometry group
    if (odometry_group.check("odometry_broadcast_port") == false)
    {
        yError() << "Missing `odometry_broadcast_port` in [ODOMETRY] group";
        return false;
    }
    m_port_broadcast_odometry_name = odometry_group.find("odometry_broadcast_port").asString();

    //opens a YARP port to receive odometry data
    std::string odom_portname = "/" + m_local_name + "/odometry:i";
    bool b1 = m_port_odometry_input.open(odom_portname.c_str());
    bool b2 = yarp::os::Network::sync(odom_portname.c_str(), false);
    bool b3 = yarp::os::Network::connect(m_port_broadcast_odometry_name.c_str(), odom_portname.c_str());
    if (b1 == false || b2 == false || b3 == false)
    {
        yError() << "Unable to initialize odometry port connection from " << m_port_broadcast_odometry_name.c_str() << "to:" << odom_portname.c_str();
        return false;
    }

    //initial location initialization
    if (initial_group.check("initial_x")) { m_initial_loc.x = initial_group.find("initial_x").asDouble(); }
    else { yError() << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y")) { m_initial_loc.y = initial_group.find("initial_y").asDouble(); }
    else { yError() << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asDouble(); }
    else { yError() << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map")) { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yError() << "missing initial_map param"; return false; }
    
    // Grab params off the param server
    m_use_map_topic = initial_group.check("use_map_topic", Value(false)).asBool();
    m_first_map_only = initial_group.check("first_map_only", Value(false)).asBool();

    double tmp;
    m_config.m_laser_min_range = amcl_group.check("laser_min_range", Value(-1.0)).asDouble();
    m_config.m_laser_max_range = amcl_group.check("laser_max_range", Value(-1.0)).asDouble();
    m_config.m_max_beams = amcl_group.check("laser_max_beams", Value(30)).asDouble();
    m_config.m_min_particles = amcl_group.check("min_particles", Value(100)).asInt();
    m_config.m_max_particles = amcl_group.check("max_particles", Value(5000)).asInt();
    m_config.m_pf_err = amcl_group.check("kld_err", Value(0.01)).asDouble();
    m_config.m_pf_z = amcl_group.check("kld_z", Value(0.99)).asDouble();
    m_config.m_alpha1 = amcl_group.check("odom_alpha1", Value(0.2)).asDouble();
    m_config.m_alpha2 = amcl_group.check("odom_alpha2", Value(0.2)).asDouble();
    m_config.m_alpha3 = amcl_group.check("odom_alpha3", Value(0.2)).asDouble();
    m_config.m_alpha4 = amcl_group.check("odom_alpha4", Value(0.2)).asDouble();
    m_config.m_alpha5 = amcl_group.check("odom_alpha5", Value(0.2)).asDouble();

    m_config.m_do_beamskip = amcl_group.check("do_beamskip", Value(false)).asBool();
    m_config.m_beam_skip_distance = amcl_group.check("beam_skip_distance", Value(0.5)).asDouble();
    m_config.m_beam_skip_threshold = amcl_group.check("beam_skip_threshold", Value(0.3)).asDouble();
    m_config.m_beam_skip_error_threshold = amcl_group.check("beam_skip_error_threshold", Value(0.9)).asDouble();

    m_config.m_z_hit = amcl_group.check("laser_z_hit", Value(0.95)).asDouble();
    m_config.m_z_short = amcl_group.check("laser_z_short", Value(0.1)).asDouble();
    m_config.m_z_max = amcl_group.check("laser_z_max", Value(0.05)).asDouble();
    m_config.m_z_rand = amcl_group.check("laser_z_rand", Value(0.05)).asDouble();
    m_config.m_sigma_hit = amcl_group.check("laser_sigma_hit", Value(0.2)).asDouble();
    m_config.m_lambda_short = amcl_group.check("laser_lambda_short", Value(0.1)).asDouble();
    m_config.m_laser_likelihood_max_dist = amcl_group.check("laser_likelihood_max_dist", Value(2.0)).asDouble();
    std::string tmp_laser_model_type = amcl_group.check("laser_model_type", Value("likelihood_field")).asString();

    m_initial_covariance_msg.resize(3, 3);
    m_initial_covariance_msg.zero();
    //the following default numbers have been taken from initialization message posted by rviz
    m_initial_covariance_msg[0][0] = 0.25;
    m_initial_covariance_msg[1][1] = 0.25;
    m_initial_covariance_msg[2][2] = 0.06853891945200942;

    if (tmp_laser_model_type == "beam")
    {
        m_laser_model_type = LASER_MODEL_BEAM;
    }
    else if (tmp_laser_model_type == "likelihood_field")
    {
        m_laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
    }
    else if (tmp_laser_model_type == "likelihood_field_prob")
    {
        m_laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
    }
    else
    {
        yWarning("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
                 tmp_laser_model_type.c_str());
                 m_laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
    }

    std::string tmp_odom_model_type = amcl_group.check("odom_model_type", Value("diff")).asString();
    if (tmp_odom_model_type == "diff")
        m_odom_model_type = ODOM_MODEL_DIFF;
    else if (tmp_odom_model_type == "omni")
        m_odom_model_type = ODOM_MODEL_OMNI;
    else if (tmp_odom_model_type == "diff-corrected")
        m_odom_model_type = ODOM_MODEL_DIFF_CORRECTED;
    else if (tmp_odom_model_type == "omni-corrected")
        m_odom_model_type = ODOM_MODEL_OMNI_CORRECTED;
    else
    {
        yWarning("Unknown odom model type \"%s\"; defaulting to diff model",
            tmp_odom_model_type.c_str());
        m_odom_model_type = ODOM_MODEL_DIFF;
    }

    m_config.m_d_thresh = amcl_group.check("update_min_d", Value(0.2)).asDouble();
    m_config.m_a_thresh = amcl_group.check("update_min_a", Value(M_PI / 6.0)).asDouble();
    m_odom_frame_id = amcl_group.check("odom_frame_id", Value("odom")).asString();
    m_base_frame_id = amcl_group.check("base_frame_id", Value("base_link")).asString();
    m_global_frame_id = amcl_group.check("global_frame_id", Value("map")).asString();
    m_resample_interval = amcl_group.check("resample_interval", Value(2)).asDouble();
     
    m_config.m_alpha_slow = amcl_group.check("recovery_alpha_slow", Value(0.001)).asDouble();
    m_config.m_alpha_fast = amcl_group.check("recovery_alpha_fast", Value(0.1)).asDouble();
    m_tf_broadcast = amcl_group.check("tf_broadcast", Value(true)).asBool();

    //get the map from the map_server
    Property map_options;
    map_options.put("device", "map2DClient");
    map_options.put("local", "/amclLocalizer"); //This is just a prefix. map2DClient will complete the port name.
    map_options.put("remote", "/mapServer");
    if (m_pMap.open(map_options) == false)
    {
        yError() << "Unable to open mapClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yError() << "Unable to open map interface";
        return false;
    }

    //get the map
    yInfo() << "Asking for map '" << m_initial_loc.map_id << "'...";
    bool b = m_iMap->get_map(m_initial_loc.map_id, m_yarp_map);
    //m_yarp_map.crop(-1, -1, -1, -1); ///@@@@@@@@@@@ do not crop for now!
    if (b)
    {
        yInfo() << "'"<< m_initial_loc.map_id <<"' received";
    }
    else
    {
        yError() << "'" << m_initial_loc.map_id << "' not found";
        return false;
    }

    m_amcl_map = convertMap(m_yarp_map);

    if (m_handler_pf != nullptr)
    {
        pf_free(m_handler_pf);
        m_handler_pf = nullptr;
    }
    m_handler_pf = pf_alloc(m_config.m_min_particles, m_config.m_max_particles,
                            m_config.m_alpha_slow, m_config.m_alpha_fast,
                           (pf_init_model_fn_t)amclLocalizerThread::uniformPoseGenerator,
                           (void *)m_amcl_map);
    m_handler_pf->pop_err = m_config.m_pf_err;
    m_handler_pf->pop_z = m_config.m_pf_z;

    // Initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = m_initial_covariance_msg[0][0];
    pf_init_pose_cov.m[0][1] = m_initial_covariance_msg[0][1];
    pf_init_pose_cov.m[1][0] = m_initial_covariance_msg[1][0];
    pf_init_pose_cov.m[1][1] = m_initial_covariance_msg[1][1];
    pf_init_pose_cov.m[2][2] = m_initial_covariance_msg[2][2];
    pf_init_pose_mean.v[0] = m_initial_loc.x;
    pf_init_pose_mean.v[1] = m_initial_loc.y;
    pf_init_pose_mean.v[2] = m_initial_loc.theta *DEG2RAD;

    pf_init(m_handler_pf, pf_init_pose_mean, pf_init_pose_cov);
    m_pf_init = false;

    // Instantiate the sensor objects
    // Odometry
    if (m_handler_odom)
    {
        delete m_handler_odom;
        m_handler_odom = 0;
    }
    m_handler_odom = new AMCLOdom();
    yAssert(m_handler_odom);
    m_handler_odom->SetModel(m_odom_model_type, m_config.m_alpha1, m_config.m_alpha2, m_config.m_alpha3, m_config.m_alpha4, m_config.m_alpha5);

    // Laser
    if (m_handler_laser)
    {
        delete m_handler_laser;
        m_handler_laser = 0;
    }
    m_handler_laser = new AMCLLaser(m_config.m_max_beams, m_amcl_map);
    yAssert(m_handler_laser);
    if (m_laser_model_type == LASER_MODEL_BEAM)
    {
        m_handler_laser->SetModelBeam(m_config.m_z_hit, m_config.m_z_short, m_config.m_z_max, m_config.m_z_rand, m_config.m_sigma_hit, m_config.m_lambda_short, 0.0);
    }
    else if (m_laser_model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    {
        yInfo("Initializing likelihood field model; this can take some time on large maps...");
        m_handler_laser->SetModelLikelihoodFieldProb(m_config.m_z_hit, m_config.m_z_rand, m_config.m_sigma_hit,
            m_config.m_laser_likelihood_max_dist,
            m_config.m_do_beamskip, m_config.m_beam_skip_distance,
            m_config.m_beam_skip_threshold, m_config.m_beam_skip_error_threshold);
        yInfo("Done initializing likelihood field model with probabilities.");
    }
    else if (m_laser_model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    {
        yInfo("Initializing likelihood field model; this can take some time on large maps...");
        m_handler_laser->SetModelLikelihoodField(m_config.m_z_hit, m_config.m_z_rand, m_config.m_sigma_hit, m_config.m_laser_likelihood_max_dist);
        yInfo("Done initializing likelihood field model.");
    }

    //opens the laser client and the corresponding interface
    Property options;
    options.put("device", "Rangefinder2DClient");
    options.put("local", "/amclLocalizer/laser:i");
    options.put("remote", m_laser_remote_port);
    if (m_pLas.open(options) == false)
    {
        yError() << "Unable to open laser driver";
        return false;
    }
    m_pLas.view(m_iLaser);
    if (m_iLaser == 0)
    {
        yError() << "Unable to open laser interface";
        return false;
    }

    if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
    {
        yError() << "Unable to obtain laser scan limits (angles)";
        return false;
    }

    if (m_iLaser->getDistanceRange(m_min_laser_distance, m_max_laser_distance) == false)
    {
        yError() << "Unable to obtain laser scan limits (distance)";
        return false;
    }

    m_laser_angle_of_view = fabs(m_min_laser_angle) + fabs(m_max_laser_angle);
    
    //@@@@ add an entry for each equipped laser device
    m_lasers.push_back(new AMCLLaser(*m_handler_laser));
    m_lasers_update.push_back(true);

    //@@@CHECK the position of this call
    this->initializeLocalization(m_initial_loc);
    return true;
}

void amclLocalizerThread::threadRelease()
{
    if (m_handler_odom)
    {
        delete m_handler_odom;
        m_handler_odom = nullptr;
    }
    if (m_handler_laser)
    {
        delete m_handler_laser;
        m_handler_laser = nullptr;
    }

    //@@@@@@@@@@@@@@must use its own alloc?
    if (m_handler_pf != nullptr)
    {
       pf_free(m_handler_pf);
       m_handler_pf = nullptr;
    }
}

pf_vector_t amclLocalizerThread::uniformPoseGenerator(void* arg)
{
    map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
    unsigned int rand_index = drand48() * free_space_indices.size();
    std::pair<int, int> free_point = free_space_indices[rand_index];
    pf_vector_t p;
    p.v[0] = MAP_WXGX(map, free_point.first);
    p.v[1] = MAP_WYGY(map, free_point.second);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
    double min_x, max_x, min_y, max_y;

    min_x = -(map->size_x * map->scale) / 2.0 + map->origin_x;
    max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
    min_y = -(map->size_y * map->scale) / 2.0 + map->origin_y;
    max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

    pf_vector_t p;

    yDebug("Generating new uniform sample");
    std::random_device rd;
    std::mt19937 gen(rd());
#if 0
    std::uniform_real_distribution<double> dis_x(min_x, max_x);
    std::uniform_real_distribution<double> dis_y(min_y, max_y);
#else
    if (max_x < min_x) std::swap(min_x, max_x);
    std::uniform_real_distribution<double> dis_x(min_x, max_x);

    if (max_y < min_y) std::swap(min_y, max_y);
    std::uniform_real_distribution<double> dis_y(min_y, max_y);
#endif
    std::uniform_real_distribution<double> dis_t(-M_PI, +M_PI);

    for (size_t check_counter=0;; check_counter++)
    {
        //p.v[0] = min_x + drand48() * (max_x - min_x);
        //p.v[1] = min_y + drand48() * (max_y - min_y);
        //p.v[2] = drand48() * 2 * M_PI - M_PI;

        p.v[0] = dis_x(gen);
        p.v[1] = dis_y(gen);
        p.v[2] = dis_t(gen);
        // Check that it's a free cell
        int i, j;
        i = MAP_GXWX(map, p.v[0]);
        j = MAP_GYWY(map, p.v[1]);
        if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1))
        {
            break;
        }

        if (check_counter > 10000)
        {
            yError() << "Deadlock detected. Problems in map data: no free cells found";
        }
    }
#endif
    return p;
}

map_t* amclLocalizerThread::convertMap(yarp::dev::MapGrid2D& yarp_map)
{
    map_t* map = map_alloc();
    yAssert(map);

    map->size_x = yarp_map.width();
    map->size_y = yarp_map.height();
    yarp_map.getResolution(map->scale);
    double x_orig;
    double y_orig;
    double t_orig;
    yarp_map.getOrigin(x_orig, y_orig, t_orig);
    map->origin_x = x_orig + (map->size_x / 2) * map->scale;
    map->origin_y = y_orig + (map->size_y / 2) * map->scale;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    yAssert(map->cells);
    //for (int i = 0; i<map->size_x * map->size_y; i++)
    for (int y = 0; y < map->size_y; y++)
        for (int x = 0; x < map->size_x; x++)
    {
        int i = y * map->size_x + x;
        double occupancy;
        yarp::dev::MapGrid2D::XYCell cell(x,y);

        yarp_map.getOccupancyData(cell, occupancy);

#if 0
        if (occupancy == 0)
            map->cells[i].occ_state = -1;
        else if (occupancy == 100)
            map->cells[i].occ_state = +1;
        else
            map->cells[i].occ_state = 0;
#else
        //@@@@check me
        if (occupancy >= 0)
        {
            if (occupancy > 50)
            {
                map->cells[i].occ_state = +1;
            }
            else
            {
                map->cells[i].occ_state = -1;
            }
        }
        else
        {
            map->cells[i].occ_state = 0;
        }
#endif

        
    }

    return map;
}

bool amclLocalizer::open(yarp::os::Searchable& config)
{
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "amclLocalizer";
    std::string file_name = "amclLocalizer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "amclLocalizer configuration: \n" << p.toString().c_str();

    double amclThreadPeriod = 1.000; //1000 ms
    if (p.check("period"))
    {
        amclThreadPeriod = p.find("period").asDouble();
    }
    thread = new amclLocalizerThread(amclThreadPeriod, p);

    if (!thread->start())
    {
        delete thread;
        thread = nullptr;
        return false;
    }

    std::string local_name = "amclLocalizer";
    Bottle general_group = p.findGroup("GENERAL");
    if (general_group.isNull()==false)
    {
        if (general_group.check("local_name")) { local_name = general_group.find("local_name").asString(); }
    }
    bool ret = rpcPort.open("/"+local_name+"/rpc");
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }

    rpcPortHandler.setInterface(this);
    rpcPort.setReader(rpcPortHandler);

    return true;
}

amclLocalizer::amclLocalizer()
{
    thread = nullptr;
}

amclLocalizer::~amclLocalizer()
{
    if (thread)
    {
        delete thread;
        thread = nullptr;
    }
}

bool amclLocalizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}
 
void amclLocalizerThread::applyInitialPose()
{
    if (m_initial_pose_hyp != nullptr && m_amcl_map != nullptr)
    {
        pf_init(m_handler_pf, m_initial_pose_hyp->pf_pose_mean, m_initial_pose_hyp->pf_pose_cov);
        m_pf_init = false;

        delete m_initial_pose_hyp;
        m_initial_pose_hyp = NULL;
    }
}