//
// Created by ste on 21/10/21.
//

#ifndef NAVIGATION_CERODOMETRY_DEVICE_H
#define NAVIGATION_CERODOMETRY_DEVICE_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/OdometryData.h>
#include <yarp/dev/IOdometry2D.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/WrapperSingle.h>

constexpr double default_period = 0.02;
#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif


/**
 * @ingroup dev_impl_fake dev_impl_navigation
 *
 * \section fakeOdometry_parameters Device description
 * \brief `CerOdometryDevice`: A device for generating a fake odometry.
 * This device will generate the odometry and then the user can retrieve it by calling `getOdometry`.
 *
 *   Parameters required by this device are:
 * | Parameter name | SubParameter            | Type    | Units          | Default Value | Required                       | Description                                                                                         | Notes |
 * |:--------------:|:-----------------------:|:-------:|:--------------:|:-------------:|:-----------------------------: |:---------------------------------------------------------------------------------------------------:|:-----:|
 * | period         |      -                  | double  | s              |   0.02        | No                             | refresh period of the broadcasted values in s                                                      | default 0.02s |
 *
 * Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device CerOdometryDevice
 * period 0.02
 * \endcode
 *
 * example of xml file
 *
 * \code{.unparsed}
 * <?xml version="1.0" encoding="UTF-8" ?>
 * <!DOCTYPE robot PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">
 * <robot name="fakeOdometry" build="2" portprefix="test" xmlns:xi="http://www.w3.org/2001/XInclude">
 *   <devices>
 *     <device xmlns:xi="http://www.w3.org/2001/XInclude" name="fakeOdometry_device" type="fakeOdometry">
 *     </device>
 *   </devices>
 * </robot>
 * \endcode
 *
 * example of command via terminal.
 *
 * \code{.unparsed}
 * yarpdev --device fakeOdometry
 * \endcode
 */

class CerOdometryDevice :
//        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        public yarp::dev::Nav2D::IOdometry2D
{
public:
    CerOdometryDevice();
    ~CerOdometryDevice() override;

//    virtual void run() override;

    // PeriodicThread
//    virtual bool threadInit() override;
//    virtual void threadRelease() override;
//
//    // odometryHandler
//    void printStats() override;
//    void compute() override;


    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IOdometry2D
    bool   getOdometry(yarp::dev::OdometryData& odom) override;
    bool   resetOdometry() override;

    // auxiliar

    void compute();

    // WrapperSingle
    bool attach(yarp::dev::PolyDriver *driver) override;
    bool detach() override;

private:
    yarp::dev::OdometryData m_odometryData;
    double base_vel_lin{0.0};
    double odom_vel_lin{0.0};

    std::mutex m_odometry_mutex;
    double last_time;
    double m_period;

    //encoder variables
    double              encL_offset{0.0};
    double              encR_offset{0.0};

    double              encL{0.0};
    double              encR{0.0};

    //measured motor velocity
    double              velL{0.0};
    double              velR{0.0};

    iCub::ctrl::AWLinEstimator      *encvel_estimator{nullptr};
    iCub::ctrl::AWLinEstimator      *encw_estimator{nullptr};

    //robot geometry
    double              geom_r{0.0};
    double              geom_L{0.0};

    yarp::sig::Vector enc;
    yarp::sig::Vector encv;

    // estimated odom
    double              traveled_distance{0.0};
    double              traveled_angle{0.0};

    //motor control interfaces
    yarp::dev::PolyDriver                      *control_board_driver{nullptr};
    yarp::dev::IEncoders                       *ienc{nullptr};

};
#endif //NAVIGATION_CERODOMETRY_DEVICE_H
