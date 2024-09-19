// Generated by gencpp from file cloud_msgs/slope.msg
// DO NOT EDIT!


#ifndef CLOUD_MSGS_MESSAGE_SLOPE_H
#define CLOUD_MSGS_MESSAGE_SLOPE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

namespace cloud_msgs
{
template <class ContainerAllocator>
struct slope_
{
  typedef slope_<ContainerAllocator> Type;

  slope_()
    : header()
    , slope()
    , slope_pos()  {
    }
  slope_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , slope(_alloc)
    , slope_pos(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::Float64_<ContainerAllocator>  _slope_type;
  _slope_type slope;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _slope_pos_type;
  _slope_pos_type slope_pos;





  typedef boost::shared_ptr< ::cloud_msgs::slope_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cloud_msgs::slope_<ContainerAllocator> const> ConstPtr;

}; // struct slope_

typedef ::cloud_msgs::slope_<std::allocator<void> > slope;

typedef boost::shared_ptr< ::cloud_msgs::slope > slopePtr;
typedef boost::shared_ptr< ::cloud_msgs::slope const> slopeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cloud_msgs::slope_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cloud_msgs::slope_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cloud_msgs::slope_<ContainerAllocator1> & lhs, const ::cloud_msgs::slope_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.slope == rhs.slope &&
    lhs.slope_pos == rhs.slope_pos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cloud_msgs::slope_<ContainerAllocator1> & lhs, const ::cloud_msgs::slope_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cloud_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cloud_msgs::slope_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cloud_msgs::slope_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cloud_msgs::slope_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cloud_msgs::slope_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cloud_msgs::slope_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cloud_msgs::slope_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cloud_msgs::slope_<ContainerAllocator> >
{
  static const char* value()
  {
    return "31b6c83548d75f9ffe937b49d92a3010";
  }

  static const char* value(const ::cloud_msgs::slope_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x31b6c83548d75f9fULL;
  static const uint64_t static_value2 = 0xfe937b49d92a3010ULL;
};

template<class ContainerAllocator>
struct DataType< ::cloud_msgs::slope_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cloud_msgs/slope";
  }

  static const char* value(const ::cloud_msgs::slope_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cloud_msgs::slope_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"std_msgs/Float64 slope\n"
"geometry_msgs/Point slope_pos\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64\n"
"float64 data\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::cloud_msgs::slope_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cloud_msgs::slope_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.slope);
      stream.next(m.slope_pos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct slope_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cloud_msgs::slope_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cloud_msgs::slope_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "slope: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.slope);
    s << indent << "slope_pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.slope_pos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLOUD_MSGS_MESSAGE_SLOPE_H