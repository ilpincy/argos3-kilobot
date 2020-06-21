#ifndef SOFTROBOT_QTUSER_FUNCTIONS_H
#define SOFTROBOT_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

using namespace argos;

class CSoftRobotLoopFunctions;

class CSoftRobotQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CSoftRobotQTUserFunctions();

   virtual ~CSoftRobotQTUserFunctions() {}

   virtual void DrawInWorld();

   void Draw(CKilobotEntity& c_entity);

private:

   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);

private:

   CSoftRobotLoopFunctions& m_cSoftRobLF;

};

#endif
