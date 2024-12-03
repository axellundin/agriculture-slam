// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from velodyne_msgs:msg/VelodynePacket.idl
// generated code does not contain a copyright notice

#include "velodyne_msgs/msg/detail/velodyne_packet__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
const rosidl_type_hash_t *
velodyne_msgs__msg__VelodynePacket__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaa, 0x15, 0x38, 0x79, 0x00, 0x60, 0xdc, 0x65,
      0x72, 0x8f, 0xbd, 0xb4, 0xaa, 0xe7, 0xb8, 0x59,
      0xc7, 0x47, 0xf1, 0x22, 0xba, 0x15, 0x6b, 0x45,
      0xf6, 0x46, 0x3b, 0x57, 0xdc, 0x1c, 0x7f, 0xc7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char velodyne_msgs__msg__VelodynePacket__TYPE_NAME[] = "velodyne_msgs/msg/VelodynePacket";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char velodyne_msgs__msg__VelodynePacket__FIELD_NAME__stamp[] = "stamp";
static char velodyne_msgs__msg__VelodynePacket__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field velodyne_msgs__msg__VelodynePacket__FIELDS[] = {
  {
    {velodyne_msgs__msg__VelodynePacket__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {velodyne_msgs__msg__VelodynePacket__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      1206,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription velodyne_msgs__msg__VelodynePacket__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
velodyne_msgs__msg__VelodynePacket__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {velodyne_msgs__msg__VelodynePacket__TYPE_NAME, 32, 32},
      {velodyne_msgs__msg__VelodynePacket__FIELDS, 2, 2},
    },
    {velodyne_msgs__msg__VelodynePacket__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Raw Velodyne LIDAR packet.\n"
  "\n"
  "builtin_interfaces/Time stamp              # packet timestamp\n"
  "uint8[1206] data        # packet contents";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
velodyne_msgs__msg__VelodynePacket__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {velodyne_msgs__msg__VelodynePacket__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 134, 134},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
velodyne_msgs__msg__VelodynePacket__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *velodyne_msgs__msg__VelodynePacket__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
