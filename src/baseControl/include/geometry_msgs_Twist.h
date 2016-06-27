// This is an automatically generated file.
// Generated from this geometry_msgs_Twist.msg definition:
//   geometry_msgs/Vector3 linear
//   geometry_msgs/Vector3 angular
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_geometry_msgs_Twist
#define YARPMSG_TYPE_geometry_msgs_Twist

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

class geometry_msgs_Twist : public yarp::os::idl::WirePortable {
public:
  geometry_msgs_Vector3 linear;
  geometry_msgs_Vector3 angular;

  geometry_msgs_Twist() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** linear ***
    if (!linear.read(connection)) return false;

    // *** angular ***
    if (!angular.read(connection)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(2)) return false;

    // *** linear ***
    if (!linear.read(connection)) return false;

    // *** angular ***
    if (!angular.read(connection)) return false;
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** linear ***
    if (!linear.write(connection)) return false;

    // *** angular ***
    if (!angular.write(connection)) return false;
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    // *** linear ***
    if (!linear.write(connection)) return false;

    // *** angular ***
    if (!angular.write(connection)) return false;
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
  typedef yarp::os::idl::BareStyle<geometry_msgs_Twist> rosStyle;
  typedef yarp::os::idl::BottleStyle<geometry_msgs_Twist> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "geometry_msgs/Vector3 linear\n\
geometry_msgs/Vector3 angular\n================================================================================\n\
MSG: geometry_msgs/Vector3\n\
float64 x\n\
float64 y\n\
float64 z";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("geometry_msgs/Twist","geometry_msgs/Twist");
    typ.addProperty("md5sum",yarp::os::Value("9f195f881246fdfa2798d1d3eebca84a"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
