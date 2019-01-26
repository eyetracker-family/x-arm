// Generated by gencpp from file reverse_kinematics/querySolutionsResponse.msg
// DO NOT EDIT!


#ifndef REVERSE_KINEMATICS_MESSAGE_QUERYSOLUTIONSRESPONSE_H
#define REVERSE_KINEMATICS_MESSAGE_QUERYSOLUTIONSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <reverse_kinematics/solutions.h>

namespace reverse_kinematics
{
template <class ContainerAllocator>
struct querySolutionsResponse_
{
  typedef querySolutionsResponse_<ContainerAllocator> Type;

  querySolutionsResponse_()
    : temp()  {
    }
  querySolutionsResponse_(const ContainerAllocator& _alloc)
    : temp(_alloc)  {
  (void)_alloc;
    }



   typedef  ::reverse_kinematics::solutions_<ContainerAllocator>  _temp_type;
  _temp_type temp;





  typedef boost::shared_ptr< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct querySolutionsResponse_

typedef ::reverse_kinematics::querySolutionsResponse_<std::allocator<void> > querySolutionsResponse;

typedef boost::shared_ptr< ::reverse_kinematics::querySolutionsResponse > querySolutionsResponsePtr;
typedef boost::shared_ptr< ::reverse_kinematics::querySolutionsResponse const> querySolutionsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace reverse_kinematics

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'reverse_kinematics': ['/home/x-arm/macaca/src/reverse_kinematics/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dfda89a7497e1f5094c64ea5d07de072";
  }

  static const char* value(const ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdfda89a7497e1f50ULL;
  static const uint64_t static_value2 = 0x94c64ea5d07de072ULL;
};

template<class ContainerAllocator>
struct DataType< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reverse_kinematics/querySolutionsResponse";
  }

  static const char* value(const ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "solutions temp\n\
\n\
\n\
================================================================================\n\
MSG: reverse_kinematics/solutions\n\
solution[] solutions\n\
\n\
================================================================================\n\
MSG: reverse_kinematics/solution\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.temp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct querySolutionsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reverse_kinematics::querySolutionsResponse_<ContainerAllocator>& v)
  {
    s << indent << "temp: ";
    s << std::endl;
    Printer< ::reverse_kinematics::solutions_<ContainerAllocator> >::stream(s, indent + "  ", v.temp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REVERSE_KINEMATICS_MESSAGE_QUERYSOLUTIONSRESPONSE_H