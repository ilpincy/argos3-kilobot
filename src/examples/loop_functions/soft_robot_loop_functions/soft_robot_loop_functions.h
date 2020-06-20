#ifndef SOFT_ROBOT_LOOP_FUNCTIONS_H
#define SOFT_ROBOT_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

using namespace argos;

class CSoftRobotLoopFunctions : public CLoopFunctions {

public:

   CSoftRobotLoopFunctions() {}
   virtual ~CSoftRobotLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void PostStep();

   virtual void Destroy();

private:

   void PlaceRobots();

   void AddSprings();
   void AddSpring(CKilobotEntity* pc_kb1,
                  CKilobotEntity* pc_kb2,
                  const CVector2& c_dir);

private:

   CKilobotEntity* GetRobot(UInt32 un_i, UInt32 un_j) {
      return m_vecRobots[un_i + un_j * m_unRobotsPerSide];
   }

private:

   std::vector<CKilobotEntity*> m_vecRobots;
   std::vector<cpConstraint*> m_vecSprings;
   
   UInt32   m_unRobotsPerSide;
   CVector2 m_cRobotsCenter;
   Real     m_fRobotsDistance;
   Real     m_fSpringRestLength;
   Real     m_fSpringStiffness;
   Real     m_fSpringDamping;
   
   CDynamics2DEngine* m_pcDyn2DEngine;
};

#endif
