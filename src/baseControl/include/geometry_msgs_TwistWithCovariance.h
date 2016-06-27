// This is an automatically generated file.
// Generated from this geometry_msgs_TwistWithCovariance.msg definition:
//   geometry_msgs/Twist twist
//   float64[36] covariance
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_geometry_msgs_TwistWithCovariance
#define YARPMSG_TYPE_geometry_msgs_TwistWithCovariance

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

class geometry_msgs_TwistWithCovariance : public yarp::os::idl::WirePortable {
public:
  geometry_msgs_Twist twist;
  std::vector<yarp::os::NetFloat64> covariance;

  geometry_msgs_TwistWithCovariance() {
    covariance.resize(36,0.0);
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** twist ***
    if (!twist.read(connection)) return false;

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

    // *** twist ***
    if (!twist.read(connection)) return false;

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
    // *** twist ***
    if (!twist.write(connection)) return false;

    // *** covariance ***
    connection.appendExternalBlock((char*)&covariance[0],sizeof(yarp::os::NetFloat64)*covariance.size());
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    // *** twist ***
    if (!twist.write(connection)) return false;

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
  typedef yarp::os::idl::BareStyle<geometry_msgs_TwistWithCovariance> rosStyle;
  typedef yarp::os::idl::BottleStyle<geometry_msgs_TwistWithCovariance> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "geometry_msgs/Twist twist\n\
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
    yarp::os::Type typ = yarp::os::Type::byName("geometry_msgs/TwistWithCovariance","geometry_msgs/TwistWithCovariance");
    typ.addProperty("md5sum",yarp::os::Value("1fe8a28e6890a4cc3ae4c3ca5c7d82e6"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
