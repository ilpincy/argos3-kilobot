/**
 * @file <ALF.cpp>
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 *
 * @brief This is the source file of the ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 */

#include "ALF.h"



CALF::CALF():
    m_fTimeForAMessage(0.05),
    m_unEnvironmentPlotUpdateFrequency(10){
}

/****************************************/
/****************************************/

void CALF::Init(TConfigurationNode& t_node) {
    /* Set the tracking type from the .argos file*/
    SetTrackingType(t_node);
    /* Get experiment variables from the .argos file*/
    GetExperimentVariables(t_node);
    /* Get the virtual environment from the .argos file */
    SetupVirtualEnvironments(t_node);
    /* Get the Kilobots entities from the space.*/
    GetKilobotsEntities();
    /* Get the initial kilobots' states */
    SetupInitialKilobotStates();
}

/****************************************/
/****************************************/

void CALF::PreStep(){
    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();
    /* Update the state of the kilobots in the space*/
    UpdateKilobotStates();
    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();
    /* Update the virtual environment*/
    UpdateVirtualEnvironments();
    /* Update the virtual environment plot*/
    PlotEnvironment();
}

/****************************************/
/****************************************/

void CALF::GetKilobotsEntities(){
    /*
     * Go through all the robots in the environment
     * and create a vector of pointers on their entities
     */
    /* Get the map of all kilobots from the space */
    CSpace::TMapPerType& mapKilobots=GetSpace().GetEntitiesByType("kilobot");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = mapKilobots.begin();
        it != mapKilobots.end();
        ++it) {
        m_tKilobotEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }
    /* Create Kilobots individual messages */
    m_tMessages=TKilobotsMessagesVector(m_tKilobotEntities.size());
}

/****************************************/
/****************************************/

void CALF::SetTrackingType(TConfigurationNode& t_tree){
    TConfigurationNode& tTrackingNode=GetNode(t_tree,"tracking");
    GetNodeAttribute(tTrackingNode, "position", m_bPositionTracking);
    GetNodeAttribute(tTrackingNode, "orientation", m_bOrientationTracking);
    GetNodeAttribute(tTrackingNode, "color", m_bColorTracking);
}

/****************************************/
/****************************************/

void CALF::UpdateKilobotStates(){
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Update the virtual states and actuators of the kilobot*/
        UpdateKilobotState(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void CALF::UpdateVirtualSensors(){
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Update the virtual sensor of a kilobot based on its current state */
        UpdateVirtualSensor(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void CALF::UpdateVirtualEnvironments(){
    /* Updates the virtual environments  based on the kilobots' states */
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Let a kilobot modify the virtual environment  */
        UpdatesVirtualEnvironmentsBasedOnKilobotState(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

CVector2 CALF::GetKilobotPosition(CKilobotEntity& c_kilobot_entity) {
    return CVector2(
                c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
}


/****************************************/
/****************************************/

CRadians CALF::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity) {
    CRadians cZAngle, cYAngle, cXAngle;
    //Calculate the orientations of the kilobot
    c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    return cZAngle;
}

/****************************************/
/****************************************/

UInt16 CALF::GetKilobotId(CKilobotEntity& c_kilobot_entity) {
    return FromString<UInt16>(c_kilobot_entity.GetControllableEntity().GetController().GetId().substr(2));
}

/****************************************/
/****************************************/

CColor CALF::GetKilobotLedColor(CKilobotEntity& c_kilobot_entity) {
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}

/****************************************/
/****************************************/

void CALF::PlotEnvironment(){
    /* Update the Floor visualization of the virtual environment every m_unEnvironmentPlotUpdateFrequency ticks*/
    if(GetSpace().GetSimulationClock()%m_unEnvironmentPlotUpdateFrequency==0)
        GetSpace().GetFloorEntity().SetChanged();
}
