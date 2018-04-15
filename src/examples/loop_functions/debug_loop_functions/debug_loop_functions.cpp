#include "debug_loop_functions.h"
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

/****************************************/
/****************************************/

void CDebugLoopFunctions::Init(TConfigurationNode& t_tree) {
   Reset();
}

/****************************************/
/****************************************/

void CDebugLoopFunctions::Reset() {
   /*
    * When the 'reset' method is called on the kilobot controller, the
    * kilobot state is destroyed and recreated. Thus, we need to
    * recreate the list of controllers and debugging info from scratch
    * as well.
    */
   m_tKBs.clear();
   /* Get the map of all kilobots from the space */
   CSpace::TMapPerType& tKBMap = GetSpace().GetEntitiesByType("kilobot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tKBMap.begin();
       it != tKBMap.end();
       ++it) {
      /* Create a pointer to the current kilobot */
      CKilobotEntity* pcKB = any_cast<CKilobotEntity*>(it->second);
      CCI_KilobotController* pcKBC = &dynamic_cast<CCI_KilobotController&>(pcKB->GetControllableEntity().GetController());
      /* Create debug info for controller */
      debug_info_t* ptDebugInfo = pcKBC->DebugInfoCreate<debug_info_t>();
      /* Append to list */
      m_tKBs.push_back(std::make_pair(pcKBC, ptDebugInfo));
   }
}

/****************************************/
/****************************************/

void CDebugLoopFunctions::PostStep() {
   /* Go through the kilobots */
   for(size_t i = 0; i < m_tKBs.size(); ++i) {
      /* Create a pointer to the kilobot state */
      kilobot_state_t* ptState = m_tKBs[i].first->GetRobotState();
      /* Print current state internal robot state */
      LOG << m_tKBs[i].first->GetId() << ": "                       << std::endl
          << "\ttx_state: "           << ptState->tx_state          << std::endl
          << "\trx_state: "           << ptState->rx_state          << std::endl
          << "\tambientlight: "       << ptState->ambientlight      << std::endl
          << "\tleft_motor: "         << ptState->left_motor        << std::endl
          << "\tright_motor: "        << ptState->right_motor       << std::endl
          << "\tcolor: "              << ptState->color             << std::endl
          << "\tgradient: "           << m_tKBs[i].second->gradient << std::endl;
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDebugLoopFunctions, "debug_loop_functions")
