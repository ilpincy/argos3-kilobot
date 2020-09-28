#include "soft_robot_loop_functions.h"
#include <argos3/plugins/robots/kilobot/simulator/dynamics2d_kilobot_model.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>

#include <sstream>
#include <list>

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.01f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

static const std::string KB_CONTROLLER  = "kbc";   // must match .argos file
static const std::string PHYSICS_ENGINE = "dyn2d"; // must match .argos file

static const Real SOFT_KILOBOT_RADIUS         = 0.02; // 2 cm
static const Real SOFT_KILOBOT_MASS           = KILOBOT_MASS + 0.014; // grams
static const Real SOFT_KILOBOT_ECCENTRICITY   = KILOBOT_ECCENTRICITY; // TODO!
static const Real SOFT_KILOBOT_SPRING_ANCHOR  = -0.014; // 1.4 cm
//static const Real        SPRING_MASS         = 0.02; // grams

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::Init(TConfigurationNode& t_tree) {
   try {
      /* Get log name and output time header*/
      GetNodeAttribute(t_tree, "output", m_strOutput);
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "Time(s)" << "\t";
      /* Get physics engine */
      m_pcDyn2DEngine = &dynamic_cast<CDynamics2DEngine&>(
         CSimulator::GetInstance().GetPhysicsEngine(PHYSICS_ENGINE));
      /* Parse attributes */
      GetNodeAttribute(t_tree, "robots_center",      m_cRobotsCenter);
      GetNodeAttribute(t_tree, "robots_per_side",    m_unRobotsPerSide);
      GetNodeAttribute(t_tree, "spring_rest_length", m_fSpringRestLength);
      GetNodeAttribute(t_tree, "spring_stiffness",   m_fSpringStiffness);
      GetNodeAttribute(t_tree, "spring_damping",     m_fSpringDamping);
      GetNodeAttribute(t_tree, "kbsr_orientation",     m_fRobotOrientation);
      /* Setup the experiment */
      m_fRobotsDistance = m_fSpringRestLength + 2 * SOFT_KILOBOT_RADIUS;
      PlaceRobots();
      AddSprings();
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::Reset() {
   CKilobotEntity* pcKB;
   for(size_t i = 0; i < m_vecRobots.size(); ++i) {
      pcKB = m_vecRobots[i];
      /* Clear the waypoint vector */
      m_tWaypoints[pcKB].clear();
      /* Add the initial position of the kilobot */
      m_tWaypoints[pcKB].push_back(pcKB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::Destroy() {
   /* Destroy all the springs (not automatically done by Chipmunk!) */
   while(!m_vecSprings.empty()) {
      cpSpaceRemoveConstraint(m_pcDyn2DEngine->GetPhysicsSpace(),
                              (cpConstraint*)m_vecSprings.back());
      cpConstraintFree((cpConstraint*)m_vecSprings.back());
      m_vecSprings.pop_back();
   }
   m_cOutput.close();
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::PostStep() {
   /* Update waypoint information */
   CKilobotEntity* pcKB;
   for(size_t i = 0; i < m_vecRobots.size(); ++i) {
      /* Create a pointer to the current kilobot */
      pcKB = m_vecRobots[i];
      /* Add the current position of the kilobot if it's sufficiently far from the last */
      if(SquareDistance(pcKB->GetEmbodiedEntity().GetOriginAnchor().Position,
                        m_tWaypoints[pcKB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcKB].push_back(pcKB->GetEmbodiedEntity().GetOriginAnchor().Position);
      }
   }
   /* Variable for calculating CoG */
  Real fCoGX = 0;
  /* Record position of each Kilobot every second */
  /* 32 must match ticks per second of simulation */
  if (GetSpace().GetSimulationClock() % 32 == 0 ) 
  {
    m_cOutput << ( GetSpace().GetSimulationClock() / 32) << "\t";
    for(size_t i = 0; i < m_vecRobots.size(); ++i) 
    {   
        pcKB = m_vecRobots[i];
        m_cOutput << pcKB->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() << "\t"
               << pcKB->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() << "\t";

               fCoGX = fCoGX + pcKB->GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    }
    m_cOutput << "\n";

    /* Kill simulation of CoG past finish line */
    if ( ( fCoGX / (m_unRobotsPerSide*m_unRobotsPerSide) )  > 1.6 )
    {
      argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
      cSimulator.Destroy();
    }

  }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::PlaceRobots() {
   try {
      Real fHalfSideLength = (m_unRobotsPerSide - 1) * m_fRobotsDistance * 0.5;
      CKilobotEntity* pcKB;
      std::ostringstream cKBId;
      CDegrees cAngleDeg = CDegrees(m_fRobotOrientation);
      CRadians cAngleRad = ToRadians(cAngleDeg);
      m_fRotSin = Sin(cAngleRad);
      m_fRotCos = Cos(cAngleRad);
      CVector3 cPos;
      CRadians cAngle = CRadians::PI_OVER_TWO + CRadians::PI_OVER_FOUR- cAngleRad;
      CQuaternion cOrientation(cAngle, CVector3::Z);
      for(size_t j = 0; j < m_unRobotsPerSide; ++j) {
         for(size_t i = 0; i < m_unRobotsPerSide; ++i) {
            /* Make id */
            cKBId.str("");
            cKBId << "kb" << (i + m_unRobotsPerSide * j + 1);
            /* Calculate position
             *
             * The ids grow with positive x and negative y
             *
             *      +x ->
             *
             * -y   1 2 3
             *  |   4 5 6
             *  v   7 8 9
             */

            Real fXcoord = m_fRobotsDistance * i - fHalfSideLength;
            Real fYcoord = -m_fRobotsDistance * j + fHalfSideLength;
            Real fRotXcoord = fXcoord * m_fRotCos + fYcoord * m_fRotSin;
            Real fRotYcoord = -fXcoord * m_fRotSin + fYcoord * m_fRotCos;
            cPos.SetX( fRotXcoord + m_cRobotsCenter.GetX());
            cPos.SetY( fRotYcoord + m_cRobotsCenter.GetY());
            /* Add robot to space */
            pcKB = new CKilobotEntity(cKBId.str(), KB_CONTROLLER, cPos, cOrientation, 0.14f);
            AddEntity(*pcKB);
            m_vecRobots.push_back(pcKB);
            /* Fix robot mass, moment, shape to match the larger version */
            CEmbodiedEntity& cBodyE =
               pcKB->GetEmbodiedEntity();
            CDynamics2DKilobotModel& cModel =
               dynamic_cast<CDynamics2DKilobotModel&>(
                  cBodyE.GetPhysicsModel(PHYSICS_ENGINE));
            cpBody* ptBody = cModel.GetBody();
            cpBodySetMass(ptBody, SOFT_KILOBOT_MASS);
            cpBodySetMoment(ptBody,
               cpMomentForCircle(SOFT_KILOBOT_MASS,
                                 0.0,
                                 2 * SOFT_KILOBOT_RADIUS,
                                 cpv(SOFT_KILOBOT_ECCENTRICITY, 0.0)));
            cpCircleShape* ptShape = (cpCircleShape*)ptBody->shapeList;
            ptShape->r = SOFT_KILOBOT_RADIUS;
            ptShape->c = cpv(SOFT_KILOBOT_ECCENTRICITY, 0.0);
            /* Create a waypoint vector */
            m_tWaypoints[pcKB] = std::vector<CVector3>();
            /* Add the initial position of the kilobot */
            m_tWaypoints[pcKB].push_back(cPos);
            m_cOutput << pcKB->GetId().c_str() << "X"  << "\t" << pcKB->GetId().c_str() << "Y" << "\t";
         }
      }
      m_cOutput << "\n";
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots", ex);
   }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::AddSprings() {
   /* Horizontal springs - e.g. KB1 to KB2 (in a 3x3 robot) */
   for(int j = 0; j < m_unRobotsPerSide; ++j) {
      for(int i = 0; i < m_unRobotsPerSide-1; ++i) {
         AddSpring(GetRobot(i,   j),
                   GetRobot(i+1, j),
                   CVector2::Y);
      }
   }
   /* Vertical springs - e.g. KB1 to KB4 (in a 3x3 robot) */
   for(int i = 0; i < m_unRobotsPerSide; ++i) {
      for(int j = 0; j < m_unRobotsPerSide-1; ++j) {
         AddSpring(GetRobot(i, j),
                   GetRobot(i, j+1),
                   -CVector2::Y);
      }
   }


   /* Addition of aproximated linear and rotational friction for each Kilobot */
    for(int i = 0; i < m_unRobotsPerSide; ++i) {
      for(int j = 0; j < m_unRobotsPerSide; ++j) {

        /*Retrieving the cp Body of the Kilobot */
        CEmbodiedEntity& cBodyE1 = GetRobot(i, j)->GetEmbodiedEntity();
        CDynamics2DKilobotModel& cModel1 = dynamic_cast<CDynamics2DKilobotModel&>(cBodyE1.GetPhysicsModel(PHYSICS_ENGINE));
        cpBody* ptBody1 = cModel1.GetBody();

        /* Creating new infinite body to apply friction from */
        cpBody* ptControlBody = cpBodyNew(INFINITY, INFINITY);

        /* Adding new aproximation of angular friction */
        cpConstraint* ptAngularConstraint =
        cpSpaceAddConstraint(m_pcDyn2DEngine->GetPhysicsSpace(),
                            cpGearJointNew(ptBody1,
                                            ptControlBody,
                                            0.0f,
                                            1.0f));
        ptAngularConstraint->maxBias = 0.0f;
        ptAngularConstraint->maxForce = 0.010f; 
      
        /* Adding new aproximation of linear friction */
        cpConstraint* ptLinearConstraint =
        cpSpaceAddConstraint(m_pcDyn2DEngine->GetPhysicsSpace(),
                            cpPivotJointNew2(ptBody1,
                                              ptControlBody,
                                              cpvzero,
                                              cpvzero));
        ptLinearConstraint->maxBias = 0.0f; 
        ptLinearConstraint->maxForce = 0.010f; 
       }
    }
}

/****************************************/
/****************************************/

void CSoftRobotLoopFunctions::AddSpring(CKilobotEntity* pc_kb1,
                                        CKilobotEntity* pc_kb2,
                                        const CVector2& c_dir) {
   /* Get the embodied entities */
   CEmbodiedEntity& cBodyE1 = pc_kb1->GetEmbodiedEntity();
   CEmbodiedEntity& cBodyE2 = pc_kb2->GetEmbodiedEntity();
   /* Get the physics models */
   CDynamics2DKilobotModel& cModel1 = dynamic_cast<CDynamics2DKilobotModel&>(cBodyE1.GetPhysicsModel(PHYSICS_ENGINE));
   CDynamics2DKilobotModel& cModel2 = dynamic_cast<CDynamics2DKilobotModel&>(cBodyE2.GetPhysicsModel(PHYSICS_ENGINE));
   /* Get the Chipmunks bodies */
   cpBody* ptBody1 = cModel1.GetBody();
   cpBody* ptBody2 = cModel2.GetBody();
   /*
    * Calculate the anchors (relative to the robot position)
    * anchor1 = dir * radius
    * anchor2 = -dir * radius
    */
   cpVect tOffset = cpvmult(
         cpv(1, c_dir.GetY()),
         SOFT_KILOBOT_SPRING_ANCHOR);
   cpVect tAnchor1 = tOffset;
   cpVect tAnchor2 = cpvneg(tOffset);

   /*Adjusting to account for offset of KB origin */
   tAnchor1 = cpvadd(tAnchor1, cpv(0.009,0));
   tAnchor2 = cpvadd(tAnchor2, cpv(0.009,0));
   
   /* Add spring between them */
   cpConstraint* ptSpring = cpDampedSpringNew(
      ptBody1, ptBody2,
      tAnchor1, tAnchor2,
      m_fSpringRestLength,
      m_fSpringStiffness,
      m_fSpringDamping);
   cpSpaceAddConstraint(m_pcDyn2DEngine->GetPhysicsSpace(),
                        ptSpring);
   m_vecSprings.push_back((cpDampedSpring*)ptSpring);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSoftRobotLoopFunctions, "soft_robot_loop_functions");
