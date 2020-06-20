/**
 * @file <ALF.h>
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 *
 * @brief This is the header file of the ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 */



#ifndef ALF_H
#define ALF_H

namespace argos {
class CSpace;
class CFloorEntity;
class CSimulator;
}

#include <math.h>


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>


#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>


using namespace argos;

/**
 * @brief The CALF class
 */

class CALF : public CLoopFunctions
{

public:

    /**
     * Class constructor.
     */
    CALF();

    /**
     * Class destructor.
     */
    virtual ~CALF(){}

    /**
     * Executes user-defined initialization logic.
     * The default implementation of this method executes the usual ALF initialization functions.
     * @param t_tree The <tt>&lt;loop_functions&gt;</tt> XML configuration tree.
     * @see Reset()
     * @see Destroy()
     */
    virtual void Init(TConfigurationNode& t_tree);

    /**
     * Executes user-defined reset logic.
     * This method should restore the state of the simulation as it was right
     * after Init() was called.
     * The default implementation of this method does nothing.
     * @see Init()
     */
    virtual void Reset(){}

    /**
     * Executes user-defined destruction logic.
     * This method should undo whatever is done in Init().
     * The default implementation of this method does nothing.
     * @see Init()
     */
    virtual void Destroy(){}

    /**
     * Executes user-defined logic right before a control step is executed.
     * This function is executed before the sensors are updated for the current time step.
     * The default implementation of this method executes the usual ALF control step.
     * @see PostStep()
     */
    virtual void PreStep();

    /**
     * Executes user-defined logic right after a control step is executed.
     * This function is executed before the actuators are updated for the next time step.
     * The default implementation of this method does nothing.
     * @see PreStep()
     */
    virtual void PostStep(){}


    /**
     * Gets a vector of all the Kilobot entities in the space
     * This function must be excuted at initialization before trying to get Kilobots states (id,position,orientation...).
     * @see init
     * @see SetupInitialKilobotStates
     */
    void GetKilobotsEntities();

    /**
     * Setups the initial state of the Kilobots in the space
     * The default implementation of this method does nothing.
     * @see GetKilobotsEntities
     */
    virtual void SetupInitialKilobotStates(){}

    /**
     * Setups the initial state of a selected kilobot entity
     * The default implementation of this method does nothing.
     * @param c_kilobot_entity A reference to the selected kilobot entity
     * @see SetupInitialKilobotStates
     */
    virtual void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity){}

    /**
     * Sets tracking type for the experiments (which kilobot states the used is interested to)
     * @param t_tree The <tt>&lt;loop_functions&gt;</tt> XML configuration tree.
     */
    void SetTrackingType(TConfigurationNode& t_tree);

    /**
     * Gets the virtual environment specified by the user from the .argos file
     * The default implementation of this method does nothing.
     * @param t_tree The <tt>&lt;loop_functions&gt;</tt> XML configuration tree.
     */
    virtual void SetupVirtualEnvironments(TConfigurationNode& t_tree){}

    /**
     * Gets variables related to the experiment from the .argos file
     * The default implementation of this method does nothing.
     * @param t_tree The <tt>&lt;loop_functions&gt;</tt> XML configuration tree.
     */
    virtual void GetExperimentVariables(TConfigurationNode& t_tree){}

    /**
     * Gets the current state of the Kilobots
     * The default implementation of this function go through the Kilobots and updates the state of each of them.
     * @see UpdateKilobotState
     */
    virtual void UpdateKilobotStates();

    /**
     * Gets the current state of a selected kilobot entity
     * @param c_kilobot_entity A reference to the selected kilobot entity
     * @see UpdateKilobotStates
     */
    virtual void UpdateKilobotState(CKilobotEntity& c_kilobot_entity){}

    /**
     * Updates the virtual sensors of the Kilobots
     * The default implementation of this function goes through the Kilobots and updates the virtual sensors of each of them.
     */
    virtual void UpdateVirtualSensors();

    /**
     * Updates the virtual sensor of a selected Kilobot entity according to its current state
     * @param c_kilobot_entity A reference to the selected kilobot entity
     * @see UpdateVirtualSensors
     */
    virtual void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity){}

    /**
     * Updates the virtual environment
     * The default implementation of this function loop through the kilobots to update the virtual environment based on their state
     * @see SetupVirtualEnvironments
     * @see UpdatesVirtualEnvironmentsBasedOnKilobotState
     */
    virtual void UpdateVirtualEnvironments();

    /**
     * Updates the virtual environment based on the state of selected kilobot entity
     * @param c_kilobot_entity A reference to the selected kilobot entity entity
     * @see SetupVirtualEnvironments
     */
    virtual void UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity& c_kilobot_entity){}

    /**
     * Get the position of a selected Kilobot entity
     * @param c_kilobot_entity A reference to the selected kilobot entity
     */
    CVector2 GetKilobotPosition(CKilobotEntity& c_kilobot_entity);

    /**
     * Get the orientation of a selected Kilobot entity
     * @param c_kilobot_entity A reference to the selected kilobot entity
     */
    CRadians GetKilobotOrientation(CKilobotEntity& c_kilobot_entity);

    /**
     * Get the ID of a selected Kilobot entity
     * @param c_kilobot_entity A reference to the selected kilobot entity
     */
    UInt16 GetKilobotId(CKilobotEntity& c_kilobot_entity);

    /**
     * Get the LedColor of a selected Kilobot entity
     * @param c_kilobot_entity A reference to the selected kilobot entity
     */
    CColor GetKilobotLedColor(CKilobotEntity& c_kilobot_entity);

    /**
     * Returns the color of the floor in the specified point.
     * This function is called if the floor entity was configured to take the loop functions
     * as source. The floor color is used by the ground sensors to calculate their readings,
     * and by the graphical visualization to create a texture to display on the arena floor.
     * @param c_pos_on_floor The position on the floor.
     * @return The color of the floor in the specified point.
     * @see CFloorEntity
     * @see PlotEnvironment
     */
    virtual CColor GetFloorColor(const CVector2& c_pos_on_floor) {
        return CColor::WHITE;
    }

    /**
     * Plots the virtual environments on the arena surface
     */
    void PlotEnvironment();


protected:

    /** List of the Kilobots in the space */
    typedef std::vector<CKilobotEntity*> TKilobotEntitiesVector;
    TKilobotEntitiesVector m_tKilobotEntities;

    /** List of the messages sent by communication entities */
    typedef std::vector<message_t> TKilobotsMessagesVector;
    TKilobotsMessagesVector m_tMessages;

    /** ALF kilobot message*/
    typedef struct {
        UInt8 m_sType:4;
        UInt16 m_sID:10;
        UInt16 m_sData:10;
    } m_tALFKilobotMessage;

    /** Tracking Flags*/
    bool m_bPositionTracking;
    bool m_bOrientationTracking;
    bool m_bColorTracking;

    /** Experiment time in seconds */
    Real m_fTimeInSeconds;

    /** Time for one kilobot message to be sent */
    Real m_fTimeForAMessage;

    /** Virtual environment update frequency in ticks*/
    UInt16 m_unEnvironmentPlotUpdateFrequency;
};

#endif
