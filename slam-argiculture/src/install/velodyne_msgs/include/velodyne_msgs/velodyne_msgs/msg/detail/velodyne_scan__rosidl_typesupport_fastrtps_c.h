// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from velodyne_msgs:msg/VelodyneScan.idl
// generated code does not contain a copyright notice
#ifndef VELODYNE_MSGS__MSG__DETAIL__VELODYNE_SCAN__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define VELODYNE_MSGS__MSG__DETAIL__VELODYNE_SCAN__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "velodyne_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "velodyne_msgs/msg/detail/velodyne_scan__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
bool cdr_serialize_velodyne_msgs__msg__VelodyneScan(
  const velodyne_msgs__msg__VelodyneScan * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
bool cdr_deserialize_velodyne_msgs__msg__VelodyneScan(
  eprosima::fastcdr::Cdr &,
  velodyne_msgs__msg__VelodyneScan * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
size_t get_serialized_size_velodyne_msgs__msg__VelodyneScan(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
size_t max_serialized_size_velodyne_msgs__msg__VelodyneScan(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
bool cdr_serialize_key_velodyne_msgs__msg__VelodyneScan(
  const velodyne_msgs__msg__VelodyneScan * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
size_t get_serialized_size_key_velodyne_msgs__msg__VelodyneScan(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
size_t max_serialized_size_key_velodyne_msgs__msg__VelodyneScan(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_velodyne_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, velodyne_msgs, msg, VelodyneScan)();

#ifdef __cplusplus
}
#endif

#endif  // VELODYNE_MSGS__MSG__DETAIL__VELODYNE_SCAN__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
