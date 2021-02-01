/*
•   Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
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
#include <localization_device_with_estimated_odometry.h>


using namespace yarp::os;

class amclLocalizer;
class amclLocalizerThread;
#define DEBUG_DATA 1

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
                     public yarp::dev::Nav2D::ILocalization2D
{
public:
    amclLocalizerThread*    m_thread;
    amclLocalizerRPCHandler m_rpcPortHandler;
    yarp::os::Port          m_rpcPort;
    std::string             m_name = "/amclLocalizer";

public:
    virtual bool open(yarp::os::Searchable& config) override;

    amclLocalizer();
    virtual ~amclLocalizer();

    virtual bool close() override;

public:

    bool   getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status) override;
    bool   getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;
    bool   getEstimatedOdometry(yarp::dev::OdometryData& odom) override;
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;
    bool   startLocalizationService() override;
    bool   stopLocalizationService() override;

};

class amclLocalizerThread : public yarp::os::PeriodicThread,
                            public localization_device_with_estimated_odometry
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Nav2D::Map2DLocation     m_initial_loc;
    yarp::dev::Nav2D::Map2DLocation     m_odometry_data;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_name = "/amclLocalizer";

    //odometry port
    std::string                  m_port_broadcast_odometry_name;
    yarp::os::BufferedPort<yarp::dev::OdometryData>  m_port_odometry_input;
    double                       m_last_odometry_data_received;

#ifdef DEBUG_DATA
    yarp::os::BufferedPort<yarp::dev::OdometryData> m_port_odometry_debug_out;
    yarp::os::BufferedPort<yarp::dev::OdometryData> m_port_pd_debug_out;
#endif

    //map interface 
    yarp::dev::PolyDriver        m_pMap;
    yarp::dev::Nav2D::IMap2D*    m_iMap;
    yarp::dev::Nav2D::MapGrid2D  m_yarp_map;

    //laser client
    yarp::dev::PolyDriver                        m_pLas;
    yarp::dev::IRangefinder2D*                   m_iLaser;
    std::vector<yarp::dev::LaserMeasurementData> m_laser_measurement_data;
    double                                       m_laser_measurement_timestamp;
    double                                       m_min_laser_angle;
    double                                       m_max_laser_angle;
    double                                       m_horizontal_resolution;
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
    bool m_pf_initialized;
    pf_vector_t m_pf_odom_pose;
    amcl_hyp_t* m_initial_pose_hyp;
    map_t* m_amcl_map;

    //all the estimated particles
    std::mutex m_particle_poses_mutex;
    std::vector<yarp::dev::Nav2D::Map2DLocation> m_particle_poses;

    //the robot most probable position
    std::mutex                          m_localization_data_mutex;
    yarp::dev::Nav2D::Map2DLocation     m_localization_data;
    yarp::dev::Nav2D::Map2DLocation     m_pf_data;

    yarp::sig::Matrix    m_initial_covariance_msg;
public:
    amclLocalizerThread(double _period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc);
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
    bool getPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses);

private:
    static pf_vector_t uniformPoseGenerator(void* arg);
    map_t* convertMap(yarp::dev::Nav2D::MapGrid2D& yarp_map);
    void updateFilter();
    void applyInitialPose();
};
