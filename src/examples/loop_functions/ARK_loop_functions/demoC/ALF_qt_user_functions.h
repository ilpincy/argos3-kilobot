#ifndef ALF_QT_USER_FUNCTIONS_H
#define ALF_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CALFQTUserFunctions : public CQTOpenGLUserFunctions {

public:

    CALFQTUserFunctions();

    virtual ~CALFQTUserFunctions() {}

    void Draw(CKilobotEntity &c_kilobot_entity);

};

#endif
