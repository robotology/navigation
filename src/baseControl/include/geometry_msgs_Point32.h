// This is an automatically generated file.
// Generated from this geometry_msgs_Point32.msg definition:
//   # This contains the position of a point in free space(with 32 bits of precision).
//   # It is recommeded to use Point wherever possible instead of Point32.  
//   # 
//   # This recommendation is to promote interoperability.  
//   #
//   # This message is designed to take up less space when sending
//   # lots of points at once, as in the case of a PointCloud.  
//   
//   float32 x
//   float32 y
//   float32 z// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_geometry_msgs_Point32
#define YARPMSG_TYPE_geometry_msgs_Point32

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"

class geometry_msgs_Point32 : public yarp::os::idl::WirePortable {
public:
  yarp::os::NetFloat32 x;
  yarp::os::NetFloat32 y;
  yarp::os::NetFloat32 z;

  geometry_msgs_Point32() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** x ***
    if (!connection.expectBlock((char*)&x,4)) return false;

    // *** y ***
    if (!connection.expectBlock((char*)&y,4)) return false;

    // *** z ***
    if (!connection.expectBlock((char*)&z,4)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3)) return false;

    // *** x ***
    x = reader.expectDouble();

    // *** y ***
    y = reader.expectDouble();

    // *** z ***
    z = reader.expectDouble();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** x ***
    connection.appendBlock((char*)&x,4);

    // *** y ***
    connection.appendBlock((char*)&y,4);

    // *** z ***
    connection.appendBlock((char*)&z,4);
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(3);

    // *** x ***
    connection.appendInt(BOTTLE_TAG_DOUBLE);
    connection.appendDouble((double)x);

    // *** y ***
    connection.appendInt(BOTTLE_TAG_DOUBLE);
    connection.appendDouble((double)y);

    // *** z ***
    connection.appendInt(BOTTLE_TAG_DOUBLE);
    connection.appendDouble((double)z);
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
  typedef yarp::os::idl::BareStyle<geometry_msgs_Point32> rosStyle;
  typedef yarp::os::idl::BottleStyle<geometry_msgs_Point32> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("geometry_msgs/Point32","geometry_msgs/Point32");
    typ.addProperty("md5sum",yarp::os::Value("cc153912f1453b708d221682bc23d9ac"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
