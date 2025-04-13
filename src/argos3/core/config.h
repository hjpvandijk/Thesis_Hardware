/*
 * ARGoS version information
 */
#define ARGOS_VERSION ""
#define ARGOS_RELEASE ""
#define ARGOS_BUILD_FOR ""

/*
 * Whether the build was done with dynamic loading or not
 */
/* #undef ARGOS_DYNAMIC_LIBRARY_LOADING */

/*
 * System-dependent extension for dynamic libraries
 */
#ifdef ARGOS_DYNAMIC_LIBRARY_LOADING
#  define ARGOS_SHARED_LIBRARY_EXTENSION ""
#  define ARGOS_MODULE_LIBRARY_EXTENSION ""
#endif

/*
 * Install prefix for ARGoS
 */
#define ARGOS_INSTALL_PREFIX "/usr/local"

/*
 * Whether on not to use double
 */
/* #undef ARGOS_USE_DOUBLE */

/*
 * Whether ARGoS was compiled with Lua support
 */
/* #undef ARGOS_WITH_LUA */

/*
 * Whether ARGoS was compiled with Qt-OpenGL support
 */
/* #undef ARGOS_COMPILE_QTOPENGL */

/*
 * Whether ARGoS was compiled with FreeImage support
 */
/* #undef ARGOS_WITH_FREEIMAGE */

/*
 * Whether to use the ARGoS threadsafe log
 */
/* #undef ARGOS_THREADSAFE_LOG */

/*
 * Compilation flags
 */
#ifndef NDEBUG
#  define ARGOS_BUILD_FLAGS "-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse -Og -g"
#else
#  define ARGOS_BUILD_FLAGS "-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse -g -O3 -DNDEBUG"
#endif
