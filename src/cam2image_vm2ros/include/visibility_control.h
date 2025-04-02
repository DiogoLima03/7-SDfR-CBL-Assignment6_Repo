// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef cam2image_vm2ros__VISIBILITY_CONTROL_H_
#define cam2image_vm2ros__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

  // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
  //     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define cam2image_vm2ros_EXPORT __attribute__((dllexport))
#define cam2image_vm2ros_IMPORT __attribute__((dllimport))
#else
#define cam2image_vm2ros_EXPORT __declspec(dllexport)
#define cam2image_vm2ros_IMPORT __declspec(dllimport)
#endif
#ifdef cam2image_vm2ros_BUILDING_DLL
#define cam2image_vm2ros_PUBLIC cam2image_vm2ros_EXPORT
#else
#define cam2image_vm2ros_PUBLIC cam2image_vm2ros_IMPORT
#endif
#define cam2image_vm2ros_PUBLIC_TYPE cam2image_vm2ros_PUBLIC
#define IMAGE_TOOLS_LOCAL
#else
#define cam2image_vm2ros_EXPORT __attribute__((visibility("default")))
#define cam2image_vm2ros_IMPORT
#if __GNUC__ >= 4
#define cam2image_vm2ros_PUBLIC __attribute__((visibility("default")))
#define cam2image_vm2ros_LOCAL __attribute__((visibility("hidden")))
#else
#define cam2image_vm2ros_PUBLIC
#define cam2image_vm2ros_LOCAL
#endif
#define cam2image_vm2ros_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // cam2image_vm2ros__VISIBILITY_CONTROL_H_
