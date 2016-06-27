// This is an automatically generated file.
// Generated from this geometry_msgs_Polygon.msg definition:
//   #A specification of a polygon where the first and last points are assumed to be connected
//   Point32[] points
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_geometry_msgs_Polygon
#define YARPMSG_TYPE_geometry_msgs_Polygon

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "geometry_msgs_Point32.h"

class geometry_msgs_Polygon : public yarp::os::idl::WirePortable {
public:
  std::vector<geometry_msgs_Point32> points;

  geometry_msgs_Polygon() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** points ***
    int len = connection.expectInt();
    points.resize(len);
    for (int i=0; i<len; i++) {
      if (!points[i].read(connection)) return false;
    }
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1)) return false;

    // *** points ***
    if (connection.expectInt()!=BOTTLE_TAG_LIST) return false;
    int len = connection.expectInt();
    points.resize(len);
    for (int i=0; i<len; i++) {
      if (!points[i].read(connection)) return false;
    }
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** points ***
    connection.appendInt(points.size());
    for (size_t i=0; i<points.size(); i++) {
      if (!points[i].write(connection)) return false;
    }
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(1);

    // *** points ***
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(points.size());
    for (size_t i=0; i<points.size(); i++) {
      if (!points[i].write(connection)) return false;
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
  typedef yarp::os::idl::BareStyle<geometry_msgs_Polygon> rosStyle;
  typedef yarp::os::idl::BottleStyle<geometry_msgs_Polygon> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
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
    yarp::os::Type typ = yarp::os::Type::byName("geometry_msgs/Polygon","geometry_msgs/Polygon");
    typ.addProperty("md5sum",yarp::os::Value("cd60a26494a087f577976f0329fa120e"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
