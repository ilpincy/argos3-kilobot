/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "pointmass3d_kilobot_model.h"
#include "kilobot_measures.h"
#include <argos3/core/utility/math/cylinder.h>

namespace argos {

   enum KILOBOT_WHEELS {
      KILOBOT_LEFT_WHEEL = 0,
      KILOBOT_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CPointMass3DKilobotModel::CPointMass3DKilobotModel(CPointMass3DEngine& c_engine,
                                                      CKilobotEntity& c_kilobot) :
      CPointMass3DModel(c_engine, c_kilobot.GetEmbodiedEntity()),
      m_cWheeledEntity(c_kilobot.GetWheeledEntity()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Register the origin anchor update method */
      RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                           &CPointMass3DKilobotModel::UpdateOriginAnchor);
      /* Get initial rotation */
      CRadians cTmp1, cTmp2;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(m_cYaw, cTmp1, cTmp2);
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotModel::Reset() {
      CPointMass3DModel::Reset();
      CRadians cTmp1, cTmp2;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(m_cYaw, cTmp1, cTmp2);
      m_fAngularVelocity = 0.0;
   }

   /****************************************/
   /****************************************/


   void CPointMass3DKilobotModel::UpdateFromEntityStatus() {
      m_cVelocity.Set((m_fCurrentWheelVelocity[KILOBOT_RIGHT_WHEEL] + m_fCurrentWheelVelocity[KILOBOT_LEFT_WHEEL])*0.5, 0.0, 0.0);
      m_cVelocity.RotateZ(m_cYaw);
      m_fAngularVelocity = (m_fCurrentWheelVelocity[KILOBOT_RIGHT_WHEEL] - m_fCurrentWheelVelocity[KILOBOT_LEFT_WHEEL]) / KILOBOT_INTERPIN_DISTANCE;
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotModel::Step() {
      m_cPosition += m_cVelocity * m_cPM3DEngine.GetPhysicsClockTick();
      m_cYaw += CRadians(m_fAngularVelocity * m_cPM3DEngine.GetPhysicsClockTick());
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotModel::CalculateBoundingBox() {
      GetBoundingBox().MinCorner.Set(
         GetEmbodiedEntity().GetOriginAnchor().Position.GetX() - KILOBOT_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetY() - KILOBOT_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
      GetBoundingBox().MaxCorner.Set(
         GetEmbodiedEntity().GetOriginAnchor().Position.GetX() + KILOBOT_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetY() + KILOBOT_RADIUS,
         GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + KILOBOT_HEIGHT);
   }

   /****************************************/
   /****************************************/

   bool CPointMass3DKilobotModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      CCylinder m_cShape(KILOBOT_RADIUS,
                         KILOBOT_HEIGHT,
                         m_cPosition,
                         CVector3::Z);
      bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
      return bIntersects;
   }

   /****************************************/
   /****************************************/

   void CPointMass3DKilobotModel::UpdateOriginAnchor(SAnchor& s_anchor) {
      s_anchor.Position = m_cPosition;
      s_anchor.Orientation = CQuaternion(m_cYaw, CVector3::Z);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_POINTMASS3D_OPERATIONS_ON_ENTITY(CKilobotEntity, CPointMass3DKilobotModel);

   /****************************************/
   /****************************************/

}
