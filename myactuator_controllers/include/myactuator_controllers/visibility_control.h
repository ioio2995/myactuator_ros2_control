#ifndef MYACTUATOR_BROADCASTER__VISIBILITY_CONTROL_H_
#define MYACTUATOR_BROADCASTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MYACTUATOR_BROADCASTER_EXPORT __attribute__((dllexport))
#define MYACTUATOR_BROADCASTER_IMPORT __attribute__((dllimport))
#else
#define MYACTUATOR_BROADCASTER_EXPORT __declspec(dllexport)
#define MYACTUATOR_BROADCASTER_IMPORT __declspec(dllimport)
#endif
#ifdef MYACTUATOR_BROADCASTER_BUILDING_DLL
#define MYACTUATOR_BROADCASTER_PUBLIC MYACTUATOR_BROADCASTER_EXPORT
#else
#define MYACTUATOR_BROADCASTER_PUBLIC MYACTUATOR_BROADCASTER_IMPORT
#endif
#define MYACTUATOR_BROADCASTER_PUBLIC_TYPE MYACTUATOR_BROADCASTER_PUBLIC
#define MYACTUATOR_BROADCASTER_LOCAL
#else
#define MYACTUATOR_BROADCASTER_EXPORT __attribute__((visibility("default")))
#define MYACTUATOR_BROADCASTER_IMPORT
#if __GNUC__ >= 4
#define MYACTUATOR_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#define MYACTUATOR_BROADCASTER_LOCAL __attribute__((visibility("hidden")))
#else
#define MYACTUATOR_BROADCASTER_PUBLIC
#define MYACTUATOR_BROADCASTER_LOCAL
#endif
#define MYACTUATOR_BROADCASTER_PUBLIC_TYPE
#endif

#endif  // MYACTUATOR_BROADCASTER__VISIBILITY_CONTROL_H_