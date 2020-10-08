#ifndef SOFT_ROBOT_LOOP_FUNCTIONS_H
#define SOFT_ROBOT_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics2d/chipmunk-physics/include/constraints/cpDampedSpring.h>

using namespace argos;

class CSoftRobotLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CKilobotEntity*, std::vector<CVector3> > TWaypointMap;

public:

   CSoftRobotLoopFunctions() {}
   virtual ~CSoftRobotLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void Destroy();

   virtual void PostStep();

   virtual bool IsExperimentFinished() {
      return m_bDone;
   }

   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

   inline const std::vector<cpDampedSpring*>& GetSprings() const {
      return m_vecSprings;
   }

   inline Real GetSpringRestLength() const {
      return m_fSpringRestLength;
   }

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

   TWaypointMap m_tWaypoints;
   
   std::vector<CKilobotEntity*> m_vecRobots;
   std::vector<cpDampedSpring*> m_vecSprings;

   std::string m_strOutput;
   std::ofstream m_cPoseData;
   std::ofstream m_cCoMData;
   std::ofstream m_cSpringData;
   std::ofstream m_cAngleData;
   
   UInt32   m_unRobotsPerSide;
   UInt32   m_unRobotsFaulty;
   CVector2 m_cRobotsCenter;
   Real     m_fRobotsDistance;
   Real     m_fSpringRestLength;
   Real     m_fSpringStiffness;
   Real     m_fSpringDamping;

   Real     m_fRobotOrientation;
   Real     m_fRotSin;
   Real     m_fRotCos;
   
   CDynamics2DEngine* m_pcDyn2DEngine;

   bool m_bDone;
};

#endif
