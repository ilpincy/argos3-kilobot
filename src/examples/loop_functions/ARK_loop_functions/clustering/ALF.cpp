/**
 * @file <ALF.h>
 *
 * @brief This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots search for a special area. When the robot find the area, ALF signal him , and he stop moving.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 */

#include "ALF.h"


/****************************************/
/*     Default LoopFunction Methods     */
/****************************************/
CALF::CALF(): CLoopFunctions(),m_unDataAcquisitionFrequency(10),
    m_unEnvironmentPlotUpdateFrequency(10),m_fTimeForAMessage(0.05){}
CALF::~CALF(){
}
void CALF::Init(TConfigurationNode& t_node) {

    /* Create random number generator */
    m_pcRNG = CRandom::CreateRNG("argos");

    /* Set the tracking type from the .argos file*/
    SetTrackingType(t_node);

    /* Get experiment variables from the .argos file*/
    GetExperimentVariables(t_node);

    /* Get the virtual environment from the .argos file */
    SetupVirtualEnvironments(t_node);

    /* Get the initial kilobots' states */
    SetupInitialKilobotsStates();

    /* Other initializations: Varibales, Log file opening... */

    // Open a log file
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}
void CALF::Reset() {

    /* Close data file */
    m_cOutput.close();

    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

}
void CALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}
void CALF::PreStep(){

    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();

    /* Update the state of the kilobots in the space*/
    UpdateKilobotsState();

    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();

    /* Update the virtual environment*/
    UpdateVirtualEnvironments();

    /* Update the virtual environment plot*/
    PlotEnvironment();

}
void CALF::PostStep(){

//    /* Log experiment's results*/
//    if(((UInt16)m_fTimeInSeconds%m_unDataAcquisitionFrequency==0)&&((m_fTimeInSeconds-(UInt16)m_fTimeInSeconds)==0)){

//        m_cOutput << (UInt16) m_fTimeInSeconds << '\t';

//        UInt16 unKilobotID;
//        CVector2 cKilobotPosition;

//        for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){

//            unKilobotID=GetKilobotId(*m_tKilobotsEntities[it]);
//            cKilobotPosition=GetKilobotPosition(*m_tKilobotsEntities[it]);

//            m_cOutput << unKilobotID << '\t' << cKilobotPosition.GetX() << '\t' << cKilobotPosition.GetY() << '\t' << (UInt16)m_vecHasFood[unKilobotID] << '\t';

//        }

//        m_cOutput << std::endl;
//    }
}
/****************************************/
/*      Kilobot Tracking Function       */
/****************************************/
CVector2 CALF::GetKilobotPosition(CKilobotEntity& c_kilobot_entity){
    CVector2 vecKilobotPosition(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKilobotPosition;
}

CRadians CALF::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity)
{

    CRadians cZAngle;
    CRadians cYAngle;
    CRadians cXAngle;

    //Calculate the orientations of the kilobot
    CQuaternion cRobotOrientations = c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation;

    cRobotOrientations.ToEulerAngles(cZAngle,cYAngle, cXAngle);

    return cZAngle;
}

UInt16 CALF::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}

CColor CALF::GetKilobotLedColor(CKilobotEntity &c_kilobot_entity){
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}

/****************************************/
/*       Initialization functions       */
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
        m_tKilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }

    /* Create Kilobots individual messages */
    m_tMessages=TKilobotsMessagesVector(m_tKilobotsEntities.size());
}

void CALF::SetupInitialKilobotsStates(){
    /* Get the Kilobots entities from the space.*/
    GetKilobotsEntities();

    tKilobotsStates.resize(m_tKilobotsEntities.size());
    tLastTimeMessaged.resize(m_tKilobotsEntities.size());
    MinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotsEntities.size()*m_fTimeForAMessage/3.0);


    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotsEntities[it]);
    }
}

void CALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    tKilobotsStates[unKilobotID]=OUTSIDE_CLUSTERING_HUB;
    tLastTimeMessaged[unKilobotID]=-1000;
}

void CALF::SetTrackingType(TConfigurationNode& t_tree){

    TConfigurationNode& tTrackingNode=GetNode(t_tree,"tracking");

    GetNodeAttribute(tTrackingNode, "position", m_bPositionTracking);

    GetNodeAttribute(tTrackingNode, "orientation", m_bOrientationTracking);

    GetNodeAttribute(tTrackingNode, "color", m_bColorTracking);

}

void CALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){

    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode=GetNode(t_tree,"environments");

    /* Get the node defining the clustering hub parametres*/
    TConfigurationNode& t_VirtualClusteringHubNode=GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position",m_sClustringHub.Centre);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius",m_sClustringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color",m_sClustringHub.Color);
}

void CALF::GetExperimentVariables(TConfigurationNode& t_tree){

    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");

    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);

    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);

    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);

    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);

}

/****************************************/
/*          Updating functions          */
/****************************************/
void CALF::UpdateKilobotsState(){
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Update the virtual states and actuators of the kilobot*/
        UpdateKilobotState(*m_tKilobotsEntities[it]);
    }
}

void CALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Update the state of the kilobots (inside or outside the clustering hub)*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
    Real fDistance = Distance(cKilobotPosition,m_sClustringHub.Centre);
    if(fDistance<(m_sClustringHub.Radius*0.9)){
        tKilobotsStates[unKilobotID]=INSIDE_CLUSTERING_HUB;
    }
    else{
        tKilobotsStates[unKilobotID]=OUTSIDE_CLUSTERING_HUB;
    }
}

void CALF::UpdateVirtualSensors(){

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Update the virtual sensor of a kilobot based on its current state */
        UpdateVirtualSensor(*m_tKilobotsEntities[it]);
    }
}

void CALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){

    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;

    /* Flag for existance of message to send*/
    bool bMessageToSend=false;

    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - tLastTimeMessaged[unKilobotID]< MinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else{
        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = (int)tKilobotsStates[unKilobotID];

        /*  Set the message sending flag to True */
        bMessageToSend=true;

        tLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if(bMessageToSend){

        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }
        // Prepare an empty ARK-type message to fill the gap in the full kilobot message
        tEmptyMessage.m_sID=1023;
        tEmptyMessage.m_sType=0;
        tEmptyMessage.m_sData=0;

        // Fill the kilobot message by the ARK-type messages

        for (int i = 0; i < 3; ++i) {

            if(i==0){
                tMessage=tKilobotMessage;
            } else{
                tMessage=tEmptyMessage;
            }

            m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
        }

        /* Sending the message */
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }

}

void CALF::UpdateVirtualEnvironments(){

    /* Updates the virtual environments  based on the kilobots' states */
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Let a kilobot modify the virtual environment  */
        UpdatesVirtualEnvironmentsBasedOnKilobotState(*m_tKilobotsEntities[it]);
    }

    /* Update virtual envirenmonts based on other rules such as time or equation. E.g.:
        if(m_fTimeInSeconds%10==0){
            m_sFoodEnv.GoalSize-=0.05;
        }
    */


}

void CALF::UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Here the virtual environment are updated based on the kilobot "kilobot_entity" state */
}

/****************************************/
/*          Drawing functions           */
/****************************************/
CColor CALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;

    Real fDistance = Distance(vec_position_on_plane,m_sClustringHub.Centre);
    if(fDistance<m_sClustringHub.Radius){
        cColor=m_sClustringHub.Color;
    }
    return cColor;
}

void CALF::PlotEnvironment(){
    /* Update the Floor visualization of the virtual environment every m_unEnvironmentPlotUpdateFrequency ticks*/
    if(GetSpace().GetSimulationClock()%m_unEnvironmentPlotUpdateFrequency==0)
        GetSpace().GetFloorEntity().SetChanged();
}

/****************************************/
/* Here Goes the user created functions */
/****************************************/


REGISTER_LOOP_FUNCTIONS(CALF, "ALF_clustering_loop_function")
