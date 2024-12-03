// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from velodyne_msgs:msg/VelodyneScan.idl
// generated code does not contain a copyright notice

#include "velodyne_msgs/msg/detail/velodyne_scan__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_velodyne_msgs
const rosidl_type_hash_t *
velodyne_msgs__msg__VelodyneScan__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4e, 0x4e, 0xdd, 0x71, 0x52, 0xe8, 0xbd, 0x60,
      0x43, 0x11, 0xd8, 0xb2, 0xe6, 0x8e, 0x4c, 0x1b,
      0x07, 0x71, 0xc8, 0xce, 0x72, 0x70, 0x0d, 0xe7,
      0x9f, 0x6d, 0x14, 0x18, 0xaa, 0x24, 0x2b, 0x3c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "velodyne_msgs/msg/detail/velodyne_packet__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
static const rosidl_type_hash_t velodyne_msgs__msg__VelodynePacket__EXPECTED_HASH = {1, {
    0xaa, 0x15, 0x38, 0x79, 0x00, 0x60, 0xdc, 0x65,
    0x72, 0x8f, 0xbd, 0xb4, 0xaa, 0xe7, 0xb8, 0x59,
    0xc7, 0x47, 0xf1, 0x22, 0xba, 0x15, 0x6b, 0x45,
    0xf6, 0x46, 0x3b, 0x57, 0xdc, 0x1c, 0x7f, 0xc7,
  }};
#endif

static char velodyne_msgs__msg__VelodyneScan__TYPE_NAME[] = "velodyne_msgs/msg/VelodyneScan";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char velodyne_msgs__msg__VelodynePacket__TYPE_NAME[] = "velodyne_msgs/msg/VelodynePacket";

// Define type names, field names, and default values
static char velodyne_msgs__msg__VelodyneScan__FIELD_NAME__header[] = "header";
static char velodyne_msgs__msg__VelodyneScan__FIELD_NAME__packets[] = "packets";

static rosidl_runtime_c__type_description__Field velodyne_msgs__msg__VelodyneScan__FIELDS[] = {
  {
    {velodyne_msgs__msg__VelodyneScan__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {velodyne_msgs__msg__VelodyneScan__FIELD_NAME__packets, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {velodyne_msgs__msg__VelodynePacket__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription velodyne_msgs__msg__VelodyneScan__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {velodyne_msgs__msg__VelodynePacket__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
velodyne_msgs__msg__VelodyneScan__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {velodyne_msgs__msg__VelodyneScan__TYPE_NAME, 30, 30},
      {velodyne_msgs__msg__VelodyneScan__FIELDS, 2, 2},
    },
    {velodyne_msgs__msg__VelodyneScan__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&velodyne_msgs__msg__VelodynePacket__EXPECTED_HASH, velodyne_msgs__msg__VelodynePacket__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = velodyne_msgs__msg__VelodynePacket__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Velodyne LIDAR scan packets.\n"
  "\n"
  "std_msgs/Header header         # standard ROS message header\n"
  "VelodynePacket[] packets        # vector of raw packets";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
velodyne_msgs__msg__VelodyneScan__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {velodyne_msgs__msg__VelodyneScan__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 149, 149},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
velodyne_msgs__msg__VelodyneScan__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *velodyne_msgs__msg__VelodyneScan__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[3] = *velodyne_msgs__msg__VelodynePacket__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
