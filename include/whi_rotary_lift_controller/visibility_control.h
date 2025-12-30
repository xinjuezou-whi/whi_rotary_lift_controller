#ifndef WHI_ROTARY_LIFT_CONTROLLER__VISIBILITY_CONTROL_H_
#define WHI_ROTARY_LIFT_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHI_ROTARY_LIFT_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define WHI_ROTARY_LIFT_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define WHI_ROTARY_LIFT_CONTROLLER_EXPORT __declspec(dllexport)
    #define WHI_ROTARY_LIFT_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHI_ROTARY_LIFT_CONTROLLER_BUILDING_LIBRARY
    #define WHI_ROTARY_LIFT_CONTROLLER_PUBLIC WHI_ROTARY_LIFT_CONTROLLER_EXPORT
  #else
    #define WHI_ROTARY_LIFT_CONTROLLER_PUBLIC WHI_ROTARY_LIFT_CONTROLLER_IMPORT
  #endif
  #define WHI_ROTARY_LIFT_CONTROLLER_PUBLIC_TYPE WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
  #define WHI_ROTARY_LIFT_CONTROLLER_LOCAL
#else
  #define WHI_ROTARY_LIFT_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define WHI_ROTARY_LIFT_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define WHI_ROTARY_LIFT_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define WHI_ROTARY_LIFT_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
    #define WHI_ROTARY_LIFT_CONTROLLER_LOCAL
  #endif
  #define WHI_ROTARY_LIFT_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // WHI_ROTARY_LIFT_CONTROLLER__VISIBILITY_CONTROL_H_
