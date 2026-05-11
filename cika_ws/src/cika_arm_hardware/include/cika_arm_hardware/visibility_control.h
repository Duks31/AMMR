#ifndef CIKA_ARM_HARDWARE__VISIBILITY_CONTROL_H_
#define CIKA_ARM_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CIKA_ARM_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define CIKA_ARM_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define CIKA_ARM_HARDWARE_EXPORT __declspec(dllexport)
    #define CIKA_ARM_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CIKA_ARM_HARDWARE_BUILDING_LIBRARY
    #define CIKA_ARM_HARDWARE_PUBLIC CIKA_ARM_HARDWARE_EXPORT
  #else
    #define CIKA_ARM_HARDWARE_PUBLIC CIKA_ARM_HARDWARE_IMPORT
  #endif
  #define CIKA_ARM_HARDWARE_PUBLIC_TYPE CIKA_ARM_HARDWARE_PUBLIC
  #define CIKA_ARM_HARDWARE_LOCAL
#else
  #define CIKA_ARM_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define CIKA_ARM_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define CIKA_ARM_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define CIKA_ARM_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CIKA_ARM_HARDWARE_PUBLIC
    #define CIKA_ARM_HARDWARE_LOCAL
  #endif
  #define CIKA_ARM_HARDWARE_PUBLIC_TYPE
#endif

#endif  // CIKA_ARM_HARDWARE__VISIBILITY_CONTROL_H_
