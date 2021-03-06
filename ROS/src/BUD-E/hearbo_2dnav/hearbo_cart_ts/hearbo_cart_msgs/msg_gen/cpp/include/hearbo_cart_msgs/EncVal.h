/* Auto-generated by genmsg_cpp for file /home/gautam/hearbo_2dnav/hearbo_cart_ts/hearbo_cart_msgs/msg/EncVal.msg */
#ifndef HEARBO_CART_MSGS_MESSAGE_ENCVAL_H
#define HEARBO_CART_MSGS_MESSAGE_ENCVAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace hearbo_cart_msgs
{
template <class ContainerAllocator>
struct EncVal_ {
  typedef EncVal_<ContainerAllocator> Type;

  EncVal_()
  : data()
  {
    data.assign(0);
  }

  EncVal_(const ContainerAllocator& _alloc)
  : data()
  {
    data.assign(0);
  }

  typedef boost::array<uint32_t, 4>  _data_type;
  boost::array<uint32_t, 4>  data;


  typedef boost::shared_ptr< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hearbo_cart_msgs::EncVal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EncVal
typedef  ::hearbo_cart_msgs::EncVal_<std::allocator<void> > EncVal;

typedef boost::shared_ptr< ::hearbo_cart_msgs::EncVal> EncValPtr;
typedef boost::shared_ptr< ::hearbo_cart_msgs::EncVal const> EncValConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::hearbo_cart_msgs::EncVal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace hearbo_cart_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hearbo_cart_msgs::EncVal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f628a1313d75e7c0f4d18e856bac80c4";
  }

  static const char* value(const  ::hearbo_cart_msgs::EncVal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf628a1313d75e7c0ULL;
  static const uint64_t static_value2 = 0xf4d18e856bac80c4ULL;
};

template<class ContainerAllocator>
struct DataType< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hearbo_cart_msgs/EncVal";
  }

  static const char* value(const  ::hearbo_cart_msgs::EncVal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# Encoder value for all the wheels of the iXs cart\n\
# data[0] : ID = 0 : Left  forward wheel\n\
# data[1] : ID = 1 : Left  back    wheel\n\
# data[2] : ID = 2 : Right back    wheel\n\
# data[3] : ID = 3 : Right forward wheel\n\
\n\
uint32[4] data\n\
\n\
";
  }

  static const char* value(const  ::hearbo_cart_msgs::EncVal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EncVal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hearbo_cart_msgs::EncVal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::hearbo_cart_msgs::EncVal_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // HEARBO_CART_MSGS_MESSAGE_ENCVAL_H

