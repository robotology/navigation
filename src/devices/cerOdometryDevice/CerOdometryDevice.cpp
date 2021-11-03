
#include "CerOdometryDevice.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Rand.h>


namespace {
    YARP_LOG_COMPONENT(CERODOMDEVICE, "yarp.device.CerOdometryDevice")
}


CerOdometryDevice::CerOdometryDevice()
{

    encvel_estimator =new iCub::ctrl::AWLinEstimator(2,1.0);
    encw_estimator = new iCub::ctrl::AWLinEstimator(1, 1.0);
    enc.resize(2);
    encv.resize(2);
    yCTrace(CERODOMDEVICE);
}

CerOdometryDevice::~CerOdometryDevice()
{
    yCTrace(CERODOMDEVICE);
    close();
}

bool CerOdometryDevice::close()
{
    detach();
    return true;
}


bool CerOdometryDevice::getOdometry(yarp::dev::OdometryData& odom)
{
    std::lock_guard lock(m_odometry_mutex);
    odom.odom_x = m_odometryData.odom_x;
    odom.odom_y = m_odometryData.odom_y;
    odom.odom_theta  = m_odometryData.odom_theta;
    odom.base_vel_x = m_odometryData.base_vel_x;
    odom.base_vel_y = m_odometryData.base_vel_y;
    odom.base_vel_theta = m_odometryData.base_vel_theta;
    odom.odom_vel_x  = m_odometryData.odom_vel_x;
    odom.odom_vel_y = m_odometryData.odom_vel_y;
    odom.odom_vel_theta = m_odometryData.odom_vel_theta;
    return true;
}

bool CerOdometryDevice::resetOdometry()
{
    ienc->getEncoder(0,&encL_offset);
    ienc->getEncoder(1,&encR_offset);
    m_odometryData.odom_x=0;
    m_odometryData.odom_y=0;
    encvel_estimator->reset();
    yCInfo(CERODOMDEVICE,"Odometry reset done");
    return true;
}

bool CerOdometryDevice::open(yarp::os::Searchable& config)
{

    yCInfo(CERODOMDEVICE,"Opening the motors interface...");

    // check period
    if (!config.check("period", "refresh period of the broadcasted values in s")) {
        yCInfo(CERODOMDEVICE) << "Using default 'period' parameter of " << default_period << "s";
    }  else {
        m_period = config.find("period").asFloat64();
    }

    // check local name (base name for ports)
    std::string localName;
    if (!config.check("local")) {
        yCWarning(CERODOMDEVICE) << "no local name found, using nothing";
    }  else {
        localName = config.find("local").asString();
    }

    //reset odometry
    resetOdometry();

    //get robot geometry
    yarp::os::Bottle geometry_group = config.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yCError(CERODOMDEVICE, "CerOdometryDevice::open Unable to find ROBOT_GEOMETRY group!");
        return false;
    }
    if (!geometry_group.check("geom_r"))
    {
        yCError(CERODOMDEVICE, "Missing param geom_r in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("geom_L"))
    {
        yCError(CERODOMDEVICE, "Missing param geom_L in [ROBOT_GEOMETRY] group");
        return false;
    }
    geom_r = geometry_group.find("geom_r").asFloat64();
    geom_L = geometry_group.find("geom_L").asFloat64();

    return true;
}

void CerOdometryDevice::compute()
{
    std::lock_guard lock(m_odometry_mutex);
    if (ienc) {
        //read the encoders (deg)
        ienc->getEncoder(0, &encL);
        ienc->getEncoder(1, &encR);

        //read the speeds (deg/s)
        ienc->getEncoderSpeed(0, &velL);
        ienc->getEncoderSpeed(1, &velR);

        //remove the offset and convert in radians
        enc[0] = (encL - encL_offset) * 0.0174532925;
        enc[1] = (encR - encR_offset) * 0.0174532925;

        //estimate the speeds
        iCub::ctrl::AWPolyElement el;
        el.data = enc;
        el.time = yarp::os::Time::now();
        encv = encvel_estimator->estimate(el);

        //compute the orientation.
        m_odometryData.odom_theta = (geom_r / geom_L) * (-enc[0] + enc[1]);

        iCub::ctrl::AWPolyElement el2;
        el2.data = yarp::sig::Vector(1, m_odometryData.odom_theta);
        el2.time = yarp::os::Time::now();
        yarp::sig::Vector vvv;
        vvv.resize(1, 0.0);
        vvv = encw_estimator->estimate(el2);

        //build the kinematics matrix
        /*yarp::sig::Matrix kin;
        kin.resize(3,2);
        kin.zero();
        kin(0, 0) = cos(odom_theta) / 2;
        kin(0, 1) = cos(odom_theta) / 2;
        kin(0, 2) = sin(odom_theta) / 2;
        kin(1, 0) = sin(odom_theta) / 2;
        kin(1, 1) = 1 / (2 * geom_L);
        kin(1, 2) = 1 / (2 * geom_L);

        yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
        yarp::sig::Vector base_cart_vels; //velocities expressed in the base reference frame
        odom_cart_vels  = kin*encv;
        base_cart_vels = kin*encv; //@@@

        base_vel_x     = base_cart_vels[1];
        base_vel_y     = base_cart_vels[0];
        base_vel_lin   = sqrt(base_vel_x*base_vel_x + base_vel_y*base_vel_y);
        base_vel_theta = base_cart_vels[2];

        odom_vel_x      = odom_cart_vels[1];
        odom_vel_y      = odom_cart_vels[0];
        odom_vel_theta  = odom_cart_vels[2];
        */


        m_odometryData.base_vel_x = geom_r / 2 * encv[0] + geom_r / 2 * encv[1];
        m_odometryData.base_vel_y = 0;
        base_vel_lin = fabs(m_odometryData.base_vel_x);
        m_odometryData.base_vel_theta = vvv[0];///-(geom_r / geom_L) * encv[0] + (geom_r / geom_L) * encv[1];
        //yCDebug() << base_vel_theta << vvv[0];


        m_odometryData.odom_vel_x = m_odometryData.base_vel_x * cos(m_odometryData.odom_theta);
        m_odometryData.odom_vel_y = m_odometryData.base_vel_x * sin(m_odometryData.odom_theta);
        odom_vel_lin = base_vel_lin;
        m_odometryData.odom_vel_theta = m_odometryData.base_vel_theta;

        //the integration step
        double period = el.time - last_time;
        m_odometryData.odom_x = m_odometryData.odom_x + (m_odometryData.odom_vel_x * period);
        m_odometryData.odom_y = m_odometryData.odom_y + (m_odometryData.odom_vel_y * period);

        //compute traveled distance (odometer)
        traveled_distance = traveled_distance + fabs(base_vel_lin * period);
        traveled_angle = traveled_angle + fabs(m_odometryData.base_vel_theta * period);

        //convert from radians back to degrees
        m_odometryData.odom_theta *= RAD2DEG;
        m_odometryData.base_vel_theta *= RAD2DEG;
        m_odometryData.odom_vel_theta *= RAD2DEG;
        traveled_angle *= RAD2DEG;

        last_time = yarp::os::Time::now();
    } else {
        yCError(CERODOMDEVICE) << "iencoder interface not valid";
    }
}

bool CerOdometryDevice::attach(yarp::dev::PolyDriver *driver) {
    if (driver->isValid())
    {
        driver->view(control_board_driver);
    } else {
        yCError(CERODOMDEVICE) << "not valid control board driver";
    }

    if (control_board_driver == nullptr)
    {
        yCError(CERODOMDEVICE, "Subdevice passed to attach method is invalid");
        return false;
    }

    // open the interfaces for the control boards
    if(!control_board_driver->view(ienc))
    {
        yCError(CERODOMDEVICE) << "iencoder device has not been viewed";
        return false;
    }
    return true;
}

bool CerOdometryDevice::detach() {
    control_board_driver = nullptr;
    ienc = nullptr;
    return true;
}

