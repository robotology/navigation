// This is an automatically generated file.
// Generated from this nav_msgs_Odometry.msg definition:
//   # This represents an estimate of a position and velocity in free space.  
//   # The pose in this message should be specified in the coordinate frame given by header.frame_id.
//   # The twist in this message should be specified in the coordinate frame given by the child_frame_id
//   Header header
//   string child_frame_id
//   geometry_msgs/PoseWithCovariance pose
//   geometry_msgs/TwistWithCovariance twist// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_nav_msgs_Odometry
#define YARPMSG_TYPE_nav_msgs_Odometry

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_Pose.h"
#include "geometry_msgs_PoseWithCovariance.h"
#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Twist.h"
#include "geometry_msgs_TwistWithCovariance.h"

class nav_msgs_Odometry : public yarp::os::idl::WirePortable {
public:
  std_msgs_Header header;
  std::string child_frame_id;
  geometry_msgs_PoseWithCovariance pose;
  geometry_msgs_TwistWithCovariance twist;

  nav_msgs_Odometry() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** header ***
    if (!header.read(connection)) return false;

    // *** child_frame_id ***
    int len = connection.expectInt();
    child_frame_id.resize(len);
    if (!connection.expectBlock((char*)child_frame_id.c_str(),len)) return false;

    // *** pose ***
    if (!pose.read(connection)) return false;

    // *** twist ***
    if (!twist.read(connection)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(4)) return false;

    // *** header ***
    if (!header.read(connection)) return false;

    // *** child_frame_id ***
    if (!reader.readString(child_frame_id)) return false;

    // *** pose ***
    if (!pose.read(connection)) return false;

    // *** twist ***
    if (!twist.read(connection)) return false;
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** header ***
    if (!header.write(connection)) return false;

    // *** child_frame_id ***
    connection.appendInt(child_frame_id.length());
    connection.appendExternalBlock((char*)child_frame_id.c_str(),child_frame_id.length());

    // *** pose ***
    if (!pose.write(connection)) return false;

    // *** twist ***
    if (!twist.write(connection)) return false;
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(4);

    // *** header ***
    if (!header.write(connection)) return false;

    // *** child_frame_id ***
    connection.appendInt(BOTTLE_TAG_STRING);
    connection.appendInt(child_frame_id.length());
    connection.appendExternalBlock((char*)child_frame_id.c_str(),child_frame_id.length());

    // *** pose ***
    if (!pose.write(connection)) return false;

    // *** twist ***
    if (!twist.write(connection)) return false;
    connection.convertTextMode();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::write;
  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<nav_msgs_Odometry> rosStyle;
  typedef yarp::os::idl::BottleStyle<nav_msgs_Odometry> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n================================================================================\n\
MSG: std_msgs/Header\n\
uint32 seq\n\
time stamp\n\
string frame_id\n================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
geometry_msgs/Pose pose\n\
float64[36] covariance\n================================================================================\n\
MSG: geometry_msgs/Pose\n\
geometry_msgs/Point position\n\
geometry_msgs/Quaternion orientation\n================================================================================\n\
MSG: geometry_msgs/Point\n\
float64 x\n\
float64 y\n\
float64 z\n================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
geometry_msgs/Twist twist\n\
float64[36] covariance\n================================================================================\n\
MSG: geometry_msgs/Twist\n\
geometry_msgs/Vector3 linear\n\
geometry_msgs/Vector3 angular\n================================================================================\n\
MSG: geometry_msgs/Vector3\n\
float64 x\n\
float64 y\n\
float64 z";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("nav_msgs/Odometry","nav_msgs/Odometry");
    typ.addProperty("md5sum",yarp::os::Value("cd5e73d190d741a2f92e81eda573aca7"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
