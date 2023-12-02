
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

#ifndef MYACTUATOR_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_
#define MYACTUATOR_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MYACTUATOR_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define MYACTUATOR_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define MYACTUATOR_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define MYACTUATOR_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef MYACTUATOR_HARDWARE_INTERFACE_BUILDING_LIBRARY
#define MYACTUATOR_HARDWARE_INTERFACE_PUBLIC MYACTUATOR_HARDWARE_INTERFACE_EXPORT
#else
#define MYACTUATOR_HARDWARE_INTERFACE_PUBLIC MYACTUATOR_HARDWARE_INTERFACE_IMPORT
#endif
#define MYACTUATOR_HARDWARE_INTERFACE_PUBLIC_TYPE MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
#define MYACTUATOR_HARDWARE_INTERFACE_LOCAL
#else
#define MYACTUATOR_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define MYACTUATOR_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define MYACTUATOR_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define MYACTUATOR_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
#define MYACTUATOR_HARDWARE_INTERFACE_LOCAL
#endif
#define MYACTUATOR_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // MYACTUATOR_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_
