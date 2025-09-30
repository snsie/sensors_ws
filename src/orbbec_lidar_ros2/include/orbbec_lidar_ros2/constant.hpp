#pragma once

#include <cstdlib>
#include <string>

#define THREAD_NUM 4

#define OB_ROS_MAJOR_VERSION 1
#define OB_ROS_MINOR_VERSION 0
#define OB_ROS_PATCH_VERSION 4

#ifndef STRINGIFY
#define STRINGIFY(arg) #arg
#endif
#ifndef VAR_ARG_STRING
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
#endif

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define OB_ROS_VERSION_STR \
  (VAR_ARG_STRING(OB_ROS_MAJOR_VERSION.OB_ROS_MINOR_VERSION.OB_ROS_PATCH_VERSION))
