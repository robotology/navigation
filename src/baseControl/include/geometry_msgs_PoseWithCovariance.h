// This is an automatically generated file.
// Generated from this geometry_msgs_PoseWithCovariance.msg definition:
//   geometry_msgs/Pose pose
//   float64[36] covariance
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_geometry_msgs_PoseWithCovariance
#define YARPMSG_TYPE_geometry_msgs_PoseWithCovariance

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_Pose.h"

class geometry_msgs_PoseWithCovariance : public yarp::os::idl::WirePortable {
public:
  geometry_msgs_Pose pose;
  std::vector<yarp::os::NetFloat64> covariance;

  geometry_msgs_PoseWithCovariance() {
    covariance.resize(36,0.0);
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** pose ***
    if (!pose.read(connection)) return false;

    // *** covariance ***
    int len = 36;
    covariance.resize(len);
    if (!connection.expectBlock((char*)&covariance[0],sizeof(yarp::os::NetFloat64)*len)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(2)) return false;

    // *** pose ***
    if (!pose.read(connection)) return false;

    // *** covariance ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    int len = connection.expectInt();
    covariance.resize(len);
    for (int i=0; i<len; i++) {
      covariance[i] = (yarp::os::NetFloat64)connection.expectDouble();
    }
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** pose ***
    if (!pose.write(connection)) return false;

    // *** covariance ***
    connection.appendExternalBlock((char*)&covariance[0],sizeof(yarp::os::NetFloat64)*covariance.size());
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    // *** pose ***
    if (!pose.write(connection)) return false;

    // *** covariance ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(covariance.size());
    for (size_t i=0; i<covariance.size(); i++) {
      connection.appendDouble((double)covariance[i]);
    }
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
  typedef yarp::os::idl::BareStyle<geometry_msgs_PoseWithCovariance> rosStyle;
  typedef yarp::os::idl::BottleStyle<geometry_msgs_PoseWithCovariance> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "geometry_msgs/Pose pose\n\
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
float64 w";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("geometry_msgs/PoseWithCovariance","geometry_msgs/PoseWithCovariance");
    typ.addProperty("md5sum",yarp::os::Value("c23e848cf1b7533a8d7c259073a97e6f"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
