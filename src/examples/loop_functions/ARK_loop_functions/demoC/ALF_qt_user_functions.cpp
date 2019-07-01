#include "ALF_qt_user_functions.h"
#include "demoC_ALF.h"

using namespace argos;

/****************************************/
/****************************************/

CALFQTUserFunctions::CALFQTUserFunctions() {
    RegisterUserFunction<CALFQTUserFunctions,CKilobotEntity>(&CALFQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CALFQTUserFunctions::Draw(CKilobotEntity& c_kilobot_entity) {
    CColor cKilobotColor=c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();

    if (cKilobotColor == CColor::RED) {
        DrawCylinder(
                    CVector3(0.015f, 0.0f, 0.03f),
                    CQuaternion(),
                    0.01f,
                    0.02f,
                    CColor::RED);//,6);
    }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CALFQTUserFunctions, "ALF_qt_user_functions")
