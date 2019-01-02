// Generated by gencpp from file RoboCompPlaneROS/PointXY.msg
// DO NOT EDIT!


#ifndef ROBOCOMPPLANEROS_MESSAGE_POINTXY_H
#define ROBOCOMPPLANEROS_MESSAGE_POINTXY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace RoboCompPlaneROS
{
template <class ContainerAllocator>
struct PointXY_
{
  typedef PointXY_<ContainerAllocator> Type;

  PointXY_()
    : x(0)
    , y(0)  {
    }
  PointXY_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)  {
  (void)_alloc;
    }



   typedef int32_t _x_type;
  _x_type x;

   typedef int32_t _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> const> ConstPtr;

}; // struct PointXY_

typedef ::RoboCompPlaneROS::PointXY_<std::allocator<void> > PointXY;

typedef boost::shared_ptr< ::RoboCompPlaneROS::PointXY > PointXYPtr;
typedef boost::shared_ptr< ::RoboCompPlaneROS::PointXY const> PointXYConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::RoboCompPlaneROS::PointXY_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace RoboCompPlaneROS

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/msg'], 'RoboCompPlaneROS': ['./src']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd7b43fd41d4c47bf5c703cc7d016709";
  }

  static const char* value(const ::RoboCompPlaneROS::PointXY_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd7b43fd41d4c47bULL;
  static const uint64_t static_value2 = 0xf5c703cc7d016709ULL;
};

template<class ContainerAllocator>
struct DataType< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
{
  static const char* value()
  {
    return "RoboCompPlaneROS/PointXY";
  }

  static const char* value(const ::RoboCompPlaneROS::PointXY_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 x\n\
int32 y\n\
";
  }

  static const char* value(const ::RoboCompPlaneROS::PointXY_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointXY_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::RoboCompPlaneROS::PointXY_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::RoboCompPlaneROS::PointXY_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOCOMPPLANEROS_MESSAGE_POINTXY_H
