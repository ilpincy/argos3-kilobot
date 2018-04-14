#include "trajectory_loop_functions.h"

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
   /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
   /* Get the map of all kilobots from the space */
   CSpace::TMapPerType& tKBMap = GetSpace().GetEntitiesByType("kilobot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tKBMap.begin();
       it != tKBMap.end();
       ++it) {
      /* Create a pointer to the current kilobot */
      CKilobotEntity* pcKB = any_cast<CKilobotEntity*>(it->second);
      /* Create a waypoint vector */
      m_tWaypoints[pcKB] = std::vector<CVector3>();
      /* Add the initial position of the kilobot */
      m_tWaypoints[pcKB].push_back(pcKB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   /* Get the map of all kilobots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("kilobot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current kilobot */
      CKilobotEntity* pcFB = any_cast<CKilobotEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the kilobot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
   /* Get the map of all kilobots from the space */
   CSpace::TMapPerType& tKBMap = GetSpace().GetEntitiesByType("kilobot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tKBMap.begin();
       it != tKBMap.end();
       ++it) {
      /* Create a pointer to the current kilobot */
      CKilobotEntity* pcKB = any_cast<CKilobotEntity*>(it->second);
      /* Add the current position of the kilobot if it's sufficiently far from the last */
      if(SquareDistance(pcKB->GetEmbodiedEntity().GetOriginAnchor().Position,
                        m_tWaypoints[pcKB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcKB].push_back(pcKB->GetEmbodiedEntity().GetOriginAnchor().Position);
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions")
