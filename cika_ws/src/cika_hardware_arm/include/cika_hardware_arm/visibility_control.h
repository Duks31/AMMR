#ifndef CIKA_HARDWARE_ARM__VISIBILITY_CONTROL_H_
#define CIKA_HARDWARE_ARM__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CIKA_HARDWARE_ARM_EXPORT __attribute__ ((dllexport))
    #define CIKA_HARDWARE_ARM_IMPORT __attribute__ ((dllimport))
  #else
    #define CIKA_HARDWARE_ARM_EXPORT __declspec(dllexport)
    #define CIKA_HARDWARE_ARM_IMPORT __declspec(dllimport)
  #endif
  #ifdef CIKA_HARDWARE_ARM_BUILDING_DLL
    #define CIKA_HARDWARE_ARM_PUBLIC CIKA_HARDWARE_ARM_EXPORT
  #else
    #define CIKA_HARDWARE_ARM_PUBLIC CIKA_HARDWARE_ARM_IMPORT
  #endif
#else
  #define CIKA_HARDWARE_ARM_EXPORT __attribute__ ((visibility ("default")))
  #define CIKA_HARDWARE_ARM_IMPORT
  #if __GNUC__ >= 4
    #define CIKA_HARDWARE_ARM_PUBLIC __attribute__ ((visibility ("default")))
    #define CIKA_HARDWARE_ARM_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define CIKA_HARDWARE_ARM_PUBLIC
    #define CIKA_HARDWARE_ARM_LOCAL
  #endif
#endif

#endif  // CIKA_HARDWARE_ARM__VISIBILITY_CONTROL_H_