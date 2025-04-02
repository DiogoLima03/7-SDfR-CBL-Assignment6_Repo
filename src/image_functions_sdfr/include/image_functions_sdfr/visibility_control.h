#ifndef IMAGE_FUNCTIONS__VISIBILITY_CONTROL_H_
#define IMAGE_FUNCTIONS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_FUNCTIONS_EXPORT __attribute__ ((dllexport))
    #define IMAGE_FUNCTIONS_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_FUNCTIONS_EXPORT __declspec(dllexport)
    #define IMAGE_FUNCTIONS_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_FUNCTIONS_BUILDING_LIBRARY
    #define IMAGE_FUNCTIONS_PUBLIC IMAGE_FUNCTIONS_EXPORT
  #else
    #define IMAGE_FUNCTIONS_PUBLIC IMAGE_FUNCTIONS_IMPORT
  #endif
  #define IMAGE_FUNCTIONS_PUBLIC_TYPE IMAGE_FUNCTIONS_PUBLIC
  #define IMAGE_FUNCTIONS_LOCAL
#else
  #define IMAGE_FUNCTIONS_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_FUNCTIONS_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_FUNCTIONS_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_FUNCTIONS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_FUNCTIONS_PUBLIC
    #define IMAGE_FUNCTIONS_LOCAL
  #endif
  #define IMAGE_FUNCTIONS_PUBLIC_TYPE
#endif

#endif  // IMAGE_FUNCTIONS__VISIBILITY_CONTROL_H_
