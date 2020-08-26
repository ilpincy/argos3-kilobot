#include "soft_robot_qtuser_functions.h"
#include "soft_robot_loop_functions.h"

/****************************************/
/****************************************/

CSoftRobotQTUserFunctions::CSoftRobotQTUserFunctions() :
   m_cSoftRobLF(dynamic_cast<CSoftRobotLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
   RegisterUserFunction<CSoftRobotQTUserFunctions,CKilobotEntity>(&CSoftRobotQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CSoftRobotQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(CSoftRobotLoopFunctions::TWaypointMap::const_iterator it = m_cSoftRobLF.GetWaypoints().begin();
       it != m_cSoftRobLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }
   /* Go through all the springs and draw them */
   for(std::vector<cpDampedSpring*>::const_iterator it = m_cSoftRobLF.GetSprings().begin();
       it != m_cSoftRobLF.GetSprings().end();
       ++it) {
      cpDampedSpring* ptSpring = (*it);
      /* Get the connected bodies */
      cpBody* ptBody1 = ptSpring->constraint.a;
      cpBody* ptBody2 = ptSpring->constraint.b;
      /* Calculate the spring anchors world positions */
      cpVect tAnchor1 = cpvadd(ptBody1->p, cpvrotate(ptSpring->anchr1, ptBody1->rot));
      cpVect tAnchor2 = cpvadd(ptBody2->p, cpvrotate(ptSpring->anchr2, ptBody2->rot));
      /* Calculate the distance */
      cpFloat fDist = cpvlength(cpvsub(tAnchor1, tAnchor2));
      /* Pick a color depending on the length of the spring
       * - GREEN -> spring is shorter than rest length
       * - RED   -> spring is longer than rest length
       */
      CColor cColor = CColor::GREEN;
      if(fDist > m_cSoftRobLF.GetSpringRestLength()) {
         cColor = CColor::RED;
      }
      /* Draw ray */
      DrawRay(CRay3(
                 CVector3(tAnchor1.x, tAnchor1.y, 0.02),
                 CVector3(tAnchor2.x, tAnchor2.y, 0.02)),
              cColor,
              3.0 // width
         );
   }

    /* Drawing Start and finish lines */
    DrawRay(CRay3(
              CVector3(0.4, 0, 0.02),
              CVector3(0.4, 2, 0.02)),
              CColor::RED,
              3.0 // width
         );
    DrawRay(CRay3(
              CVector3(1.6, 0, 0.02),
              CVector3(1.6, 2, 0.02)),
              CColor::RED,
              3.0 // width
         );
}

/****************************************/
/****************************************/

void CSoftRobotQTUserFunctions::Draw(CKilobotEntity& c_entity) {
   DrawText(CVector3(0.0, 0.0, 0.1),   // position
            c_entity.GetId()); // text

  /* Drawing a representation of the circlet on each KB */
  DrawCylinder(
        CVector3(0.009f, 0.0f, 0.0304f), 
        CQuaternion(),
        0.02f,
        0.006f,
        CColor::WHITE);
}

/****************************************/
/****************************************/

void CSoftRobotQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]),
                 CColor::BLACK);
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CSoftRobotQTUserFunctions, "soft_robot_qtuser_functions")
