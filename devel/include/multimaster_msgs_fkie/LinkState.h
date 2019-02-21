// Generated by gencpp from file multimaster_msgs_fkie/LinkState.msg
// DO NOT EDIT!


#ifndef MULTIMASTER_MSGS_FKIE_MESSAGE_LINKSTATE_H
#define MULTIMASTER_MSGS_FKIE_MESSAGE_LINKSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace multimaster_msgs_fkie
{
template <class ContainerAllocator>
struct LinkState_
{
  typedef LinkState_<ContainerAllocator> Type;

  LinkState_()
    : destination()
    , quality(0.0)  {
    }
  LinkState_(const ContainerAllocator& _alloc)
    : destination(_alloc)
    , quality(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _destination_type;
  _destination_type destination;

   typedef float _quality_type;
  _quality_type quality;





  typedef boost::shared_ptr< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> const> ConstPtr;

}; // struct LinkState_

typedef ::multimaster_msgs_fkie::LinkState_<std::allocator<void> > LinkState;

typedef boost::shared_ptr< ::multimaster_msgs_fkie::LinkState > LinkStatePtr;
typedef boost::shared_ptr< ::multimaster_msgs_fkie::LinkState const> LinkStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace multimaster_msgs_fkie

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'multimaster_msgs_fkie': ['/home/jackson/Development/HARE/src/multimaster_fkie/multimaster_msgs_fkie/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d3fe0aab52336c0bd0bfea55b131c66e";
  }

  static const char* value(const ::multimaster_msgs_fkie::LinkState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd3fe0aab52336c0bULL;
  static const uint64_t static_value2 = 0xd0bfea55b131c66eULL;
};

template<class ContainerAllocator>
struct DataType< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multimaster_msgs_fkie/LinkState";
  }

  static const char* value(const ::multimaster_msgs_fkie::LinkState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string destination\n\
float32 quality\n\
";
  }

  static const char* value(const ::multimaster_msgs_fkie::LinkState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.destination);
      stream.next(m.quality);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LinkState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multimaster_msgs_fkie::LinkState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::multimaster_msgs_fkie::LinkState_<ContainerAllocator>& v)
  {
    s << indent << "destination: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.destination);
    s << indent << "quality: ";
    Printer<float>::stream(s, indent + "  ", v.quality);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MULTIMASTER_MSGS_FKIE_MESSAGE_LINKSTATE_H
