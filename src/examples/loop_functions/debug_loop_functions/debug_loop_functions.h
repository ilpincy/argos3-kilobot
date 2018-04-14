/*
 * This class show how to retrieve information from a Kilobot and how
 * to interact with it.
 * 
 * This loop functions work in conjunction with the debug behavior in
 * src/examples/behaviors/debug.c.
 */

#ifndef DEBUG_LOOP_FUNCTIONS_H
#define DEBUG_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

// A forward declaration for the kilobot controller.
// Just using this makes compilation lighter at this point.
class CCI_KilobotController;

////////////////////////////////////////
// DEBUGGING INFORMATION
//
// This is where the struct debug_info_t is defined.
#include <examples/behaviors/test_debug.h>
//
////////////////////////////////////////

using namespace argos;

class CDebugLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CKilobotEntity*, std::vector<CVector3> > TWaypointMap;
   
public:

   virtual ~CDebugLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();

private:

   ////////////////////////////////////////
   // DEBUGGING INFORMATION
   //
   // This is an efficient way to store both controllers and debug information.
   std::vector< std::pair<CCI_KilobotController*, debug_info_t*> > m_tKBs;
   //
   ////////////////////////////////////////

};

#endif
