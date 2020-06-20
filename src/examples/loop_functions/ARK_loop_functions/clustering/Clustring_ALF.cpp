/**
 * This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots search for a special area. When the robot finds the area, ALF signals him, and he stops moving.
 *
 * Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 */

#include "Clustring_ALF.h"

/****************************************/
/****************************************/

CClusteringALF::CClusteringALF() :
    m_unDataAcquisitionFrequency(10){
}

/****************************************/
/****************************************/

CClusteringALF::~CClusteringALF(){
}

/****************************************/
/****************************************/

void CClusteringALF::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CClusteringALF::Reset() {
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CClusteringALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CClusteringALF::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void CClusteringALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = OUTSIDE_CLUSTERING_HUB;
    m_vecLastTimeMessaged[unKilobotID] = -1000;
}

/****************************************/
/****************************************/

void CClusteringALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    /* Get the node defining the clustering hub parametres*/
    TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position", m_sClusteringHub.Center);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius", m_sClusteringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color", m_sClusteringHub.Color);
}

/****************************************/
/****************************************/

void CClusteringALF::GetExperimentVariables(TConfigurationNode& t_tree){
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
/****************************************/

void CClusteringALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Update the state of the kilobots (inside or outside the clustering hub)*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
    Real fDistance = Distance(cKilobotPosition, m_sClusteringHub.Center);
    if(fDistance<(m_sClusteringHub.Radius*0.9)){
        m_vecKilobotStates[unKilobotID]=INSIDE_CLUSTERING_HUB;
    }
    else{
        m_vecKilobotStates[unKilobotID]=OUTSIDE_CLUSTERING_HUB;
    }
}

/****************************************/
/****************************************/

void CClusteringALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Flag for existance of message to send*/
    bool bMessageToSend=false;
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else{
        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID];
        /*  Set the message sending flag to True */
        bMessageToSend=true;
        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
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
            if( i == 0){
                tMessage = tKilobotMessage;
            } else{
                tMessage = tEmptyMessage;
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

/****************************************/
/****************************************/

CColor CClusteringALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    Real fDistance = Distance(vec_position_on_plane,m_sClusteringHub.Center);
    if(fDistance<m_sClusteringHub.Radius){
        cColor=m_sClusteringHub.Color;
    }
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CClusteringALF, "ALF_clustering_loop_function")
