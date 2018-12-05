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
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/IMap2D.h>
#include <cmath>

#include "./amcl/map/map.h"
#include "./amcl/pf/pf.h"
#include "./amcl/sensors/amcl_odom.h"
#include "./amcl/sensors/amcl_laser.h"

using namespace yarp::os;

class amclLocalizer;
class amclLocalizerThread;

// Pose hypothesis
typedef struct
{
    // Total weight (weights sum to 1)
    double weight;

    // Mean of pose estimate
    pf_vector_t pf_pose_mean;

    // Covariance of pose estimate
    pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

class amclLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    amclLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    amclLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(amclLocalizer* iface);
};

class amclLocalizer : public yarp::dev::DeviceDriver,
                     public yarp::dev::ILocalization2D
{
public:
    amclLocalizerThread*    thread;
    amclLocalizerRPCHandler rpcPortHandler;
    yarp::os::Port          rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    amclLocalizer();
    virtual ~amclLocalizer();

    virtual bool close() override;

public:
    /**
    * Gets the current status of the localization task.
    * @return true/false
    */
    bool   getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status) override;

    /**
    * Gets a set of pose estimates computed by the localization algorithm.
    * @return true/false
    */
    bool   getEstimatedPoses(std::vector<yarp::dev::Map2DLocation>& poses) override;

    /**
    * Gets the current position of the robot w.r.t world reference frame
    * @param loc the location of the robot
    * @return true/false
    */
    bool   getCurrentPosition(yarp::dev::Map2DLocation& loc) override;

    /**
    * Sets the initial pose for the localization algorithm which estimates the current position of the robot w.r.t world reference frame.
    * @param loc the location of the robot
    * @return true/false
    */
    bool   setInitialPose(yarp::dev::Map2DLocation& loc) override;
};

class amclLocalizerThread : public yarp::os::PeriodicThread
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Map2DLocation     m_initial_loc;
    yarp::dev::Map2DLocation     m_odometry_data;
    yarp::os::Mutex              m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_local_name;

    //odometry port
    std::string                  m_port_broadcast_odometry_name;
    yarp::os::BufferedPort<yarp::sig::Vector>  m_port_odometry_input;
    double                       m_last_odometry_data_received;

    //map interface 
    yarp::dev::PolyDriver        m_pMap;
    yarp::dev::IMap2D*           m_iMap;
    yarp::dev::MapGrid2D         m_yarp_map;

    //laser client
    yarp::dev::PolyDriver                        m_pLas;
    yarp::dev::IRangefinder2D*                   m_iLaser;
    std::vector<yarp::dev::LaserMeasurementData> m_laser_measurement_data;
    double                                       m_laser_measurement_timestamp;
    double                                       m_min_laser_angle;
    double                                       m_max_laser_angle;
    double                                       m_min_laser_distance;
    double                                       m_max_laser_distance;
    double                                       m_laser_angle_of_view;
    std::string                                  m_laser_remote_port;

    bool m_use_map_topic;
    bool m_first_map_only;

    struct
    {
        double m_laser_min_range;
        double m_laser_max_range;
        double m_max_beams;
        int    m_min_particles;
        int    m_max_particles;
        double m_pf_err;
        double m_pf_z;
        double m_alpha1;
        double m_alpha2;
        double m_alpha3;
        double m_alpha4;
        double m_alpha5;
        bool   m_do_beamskip;
        double m_beam_skip_distance;
        double m_beam_skip_threshold;
        double m_beam_skip_error_threshold;
        double m_z_hit;
        double m_z_short;
        double m_z_max;
        double m_z_rand;
        double m_sigma_hit;
        double m_lambda_short;
        double m_laser_likelihood_max_dist;
        double m_alpha_slow;
        double m_alpha_fast;
        double m_d_thresh;
        double m_a_thresh;
    } m_config;

    amcl::laser_model_t m_laser_model_type;
    amcl::odom_model_t m_odom_model_type;
    std::string m_odom_frame_id;
    std::string m_base_frame_id;
    std::string m_global_frame_id;
    int m_resample_interval;
    int m_resample_count;
    std::vector< bool > m_lasers_update;
    std::vector< amcl::AMCLLaser* > m_lasers;

    bool m_tf_broadcast;

    amcl::AMCLOdom*  m_handler_odom;
    amcl::AMCLLaser* m_handler_laser;
    bool             m_force_update;

    pf_t* m_handler_pf;
    bool m_pf_init;
    pf_vector_t m_pf_odom_pose;
    amcl_hyp_t* m_initial_pose_hyp;
    map_t* m_amcl_map;

    //all the estimated particles
    yarp::os::Mutex m_particle_poses_mutex;
    std::vector<yarp::dev::Map2DLocation> m_particle_poses;

    //the robot most probable position
    yarp::os::Mutex m_localization_data_mutex;
    yarp::dev::Map2DLocation     m_localization_data;

public:
    amclLocalizerThread(double _period, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    bool initializeLocalization(yarp::dev::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Map2DLocation& loc);
    bool getPoses(std::vector<yarp::dev::Map2DLocation>& poses);

private:
    static pf_vector_t uniformPoseGenerator(void* arg);
    map_t* amclLocalizerThread::convertMap(yarp::dev::MapGrid2D& yarp_map);
    void updateFilter();
    void applyInitialPose();
};
