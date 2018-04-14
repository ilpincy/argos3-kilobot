#ifndef TEST_DEBUG_H
#define TEST_DEBUG_H

#ifdef ARGOS_simulator_BUILD

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

////////////////////////////////////////
// DEBUGGING INFORMATION
//
// You must define a struct called 'debug_info_t'
//
// The name 'debug_info_t' is mandatory
// The content of the struct is whatever you want
typedef struct {
   int gradient; // value of robot gradient
} debug_info_t;
//
////////////////////////////////////////

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif // ARGOS_simulator_BUILD

#endif // TEST_DEBUG_H
