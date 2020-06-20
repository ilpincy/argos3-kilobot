#include "soft_robot_loop_functions.h"
#include <argos3/plugins/robots/kilobot/simulator/dynamics2d_kilobot_model.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>

#include <sstream>
#include <list>

/****************************************/
/****************************************/

static const std::string KB_CONTROLLER       = "kbc";
static const std::string PHYSICS_ENGINE      = "dyn2d";
static const Real        SOFT_KILOBOT_RADIUS = KILOBOT_RADIUS;

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::Init(TConfigurationNode& t_tree) {
   try {
      /* Get physics engine */
      m_pcDyn2DEngine = &dynamic_cast<CDynamics2DEngine&>(
         CSimulator::GetInstance().GetPhysicsEngine(PHYSICS_ENGINE));
      /* Parse attributes */
      GetNodeAttribute(t_tree, "robots_center",      m_cRobotsCenter);
      GetNodeAttribute(t_tree, "robots_per_side",    m_unRobotsPerSide);
      GetNodeAttribute(t_tree, "robots_distance",    m_fRobotsDistance);
      GetNodeAttribute(t_tree, "spring_stiffness",   m_fSpringStiffness);
      GetNodeAttribute(t_tree, "spring_damping",     m_fSpringDamping);
      /* Setup the experiment */
      m_fSpringRestLength = m_fRobotsDistance - 2 * SOFT_KILOBOT_RADIUS;
      PlaceRobots();
      AddSprings();
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::PostStep() {
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::Destroy() {
   /* Destroy all the springs (not automatically done by Chipmunk!) */
   while(!m_vecSprings.empty()) {
      cpSpaceRemoveConstraint(m_pcDyn2DEngine->GetPhysicsSpace(),
                              m_vecSprings.back());
      cpConstraintFree(m_vecSprings.back());
      m_vecSprings.pop_back();
   }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::PlaceRobots() {
   try {
      Real fHalfSideLength = m_unRobotsPerSide * m_fRobotsDistance * 0.5;
      CKilobotEntity* pcKB;
      std::ostringstream cKBId;
      CVector3 cPos;
      for(size_t i = 0; i < m_unRobotsPerSide; ++i) {
         for(size_t j = 0; j < m_unRobotsPerSide; ++j) {
            /* Make id */
            cKBId.str("");
            printf("[DEBUG] i = %zu, j = %zu, idx = %zu\n", i, j, (i + m_unRobotsPerSide * j));
            cKBId << "kb" << (i + m_unRobotsPerSide * j);
            printf("[DEBUG] cKBId = %s\n", cKBId.str().c_str());
            /* Calculate position
             *
             * The ids grow with positive x and negative y
             *
             *      +x ->
             *
             * -y   0 1 2
             *  |   3 4 5
             *  v   6 7 8
             */
            cPos.SetX( m_fRobotsDistance * i - fHalfSideLength + m_cRobotsCenter.GetX());
            cPos.SetY(-m_fRobotsDistance * j + fHalfSideLength + m_cRobotsCenter.GetY());
            printf("[DEBUG] x = %f, y = %f\n", cPos.GetX(), cPos.GetY());
            /* Add robot to space */
            pcKB = new CKilobotEntity(cKBId.str(), KB_CONTROLLER, cPos);
            AddEntity(*pcKB);
            m_vecRobots.push_back(pcKB);
         }
      }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots", ex);
   }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::AddSprings() {
   /* Horizontal springs */
   for(int j = 0; j < m_unRobotsPerSide; ++j) {
      for(int i = 0; i < m_unRobotsPerSide-1; ++i) {
         AddSpring(GetRobot(i,   j),
                   GetRobot(i+1, j),
                   CVector2::X);
      }
   }
   /* Vertical springs */
   for(int i = 0; i < m_unRobotsPerSide; ++i) {
      for(int j = 0; j < m_unRobotsPerSide-1; ++j) {
         AddSpring(GetRobot(i, j),
                   GetRobot(i, j+1),
                   -CVector2::Y);
      }
   }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::AddSpring(CKilobotEntity* pc_kb1,
                                        CKilobotEntity* pc_kb2,
                                        const CVector2& c_dir) {
   /* Get the embodied entities */
   CEmbodiedEntity* pcBodyE1 = &pc_kb1->GetEmbodiedEntity();
   CEmbodiedEntity* pcBodyE2 = &pc_kb2->GetEmbodiedEntity();
   /* Get the physics models */
   CDynamics2DKilobotModel* pcModel1 = &dynamic_cast<CDynamics2DKilobotModel&>(pcBodyE1->GetPhysicsModel(PHYSICS_ENGINE));
   CDynamics2DKilobotModel* pcModel2 = &dynamic_cast<CDynamics2DKilobotModel&>(pcBodyE2->GetPhysicsModel(PHYSICS_ENGINE));
   /* Get the Chipmunks bodies */
   cpBody* pcBody1 = pcModel1->GetBody();
   cpBody* pcBody2 = pcModel2->GetBody();
   /*
    * Calculate the anchors (relative to the robot position)
    * anchor1 = dir * radius
    * anchor2 = -dir * radius
    */
   cpVect tOffset = cpvmult(
         cpv(c_dir.GetX(), c_dir.GetY()),
         SOFT_KILOBOT_RADIUS);
   cpVect tAnchor1 = tOffset;
   cpVect tAnchor2 = cpvneg(tOffset);
   /* Add spring between them */
   cpConstraint* ptSpring = cpDampedSpringNew(
      pcBody1, pcBody2,
      tAnchor1, tAnchor2,
      m_fSpringRestLength,
      m_fSpringStiffness,
      m_fSpringDamping);
   cpSpaceAddConstraint(m_pcDyn2DEngine->GetPhysicsSpace(),
                        ptSpring);
   m_vecSprings.push_back(ptSpring);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSoftRobotLoopFunctions, "soft_robot_loop_functions");
