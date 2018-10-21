#include "kilobot_communication_medium.h"
#include "kilobot_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotCommunicationMedium::CKilobotCommunicationMedium() :
      m_pcKilobotIndex(NULL),
      m_pcGridUpdateOperation(NULL),
      m_pcRNG(NULL),
      m_fRxProb(0.0),
      m_bIgnoreConflicts(false)
   {
   }

   /****************************************/
   /****************************************/

   CKilobotCommunicationMedium::~CKilobotCommunicationMedium() {
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         /* Get the arena center and size */
         CVector3 cArenaCenter;
         CVector3 cArenaSize;
         TConfigurationNode& tArena = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena");
         GetNodeAttribute(tArena, "size", cArenaSize);
         GetNodeAttributeOrDefault(tArena, "center", cArenaCenter, cArenaCenter);
         /* Create the positional index for embodied entities */
         UInt32 unXCells = Ceil(cArenaSize.GetX() / (KILOBOT_RADIUS+KILOBOT_RADIUS));
         UInt32 unYCells = Ceil(cArenaSize.GetY() / (KILOBOT_RADIUS+KILOBOT_RADIUS));
         CGrid<CKilobotCommunicationEntity>* pcGrid = new CGrid<CKilobotCommunicationEntity>(
            cArenaCenter - cArenaSize * 0.5f, cArenaCenter + cArenaSize * 0.5f,
            unXCells, unYCells, 1);
         m_pcGridUpdateOperation = new CKilobotCommunicationEntityGridEntityUpdater(*pcGrid);
         pcGrid->SetUpdateEntityOperation(m_pcGridUpdateOperation);
         m_pcKilobotIndex = pcGrid;
         /* Set probability of receiving a message */
         GetNodeAttributeOrDefault(t_tree, "message_drop_prob", m_fRxProb, m_fRxProb);
         m_fRxProb = 1.0 - m_fRxProb;
         /* Create random number generator */
         m_pcRNG = CRandom::CreateRNG("argos");
         /* Whether or not to ignore conflicts due to channel congestion */
         GetNodeAttributeOrDefault(t_tree, "ignore_conflicts", m_bIgnoreConflicts, m_bIgnoreConflicts);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error in initialization of the range-and-bearing medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::PostSpaceInit() {
      Update();
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::Reset() {
      /* Reset positional index of Kilobot entities */
      m_pcKilobotIndex->Reset();
      /* Delete adjacency matrix */
      for(TAdjacencyMatrix::iterator it = m_tCommMatrix.begin();
          it != m_tCommMatrix.end();
          ++it) {
         it->second.clear();
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::Destroy() {
      delete m_pcKilobotIndex;
      if(m_pcGridUpdateOperation != NULL)
         delete m_pcGridUpdateOperation;
   }

   /****************************************/
   /****************************************/

   static size_t HashKilobotPair(const std::pair<CKilobotCommunicationEntity*, CKilobotCommunicationEntity*>& c_pair) {
      return
         reinterpret_cast<size_t>(c_pair.first) ^
         reinterpret_cast<size_t>(c_pair.second);
   }

   void CKilobotCommunicationMedium::Update() {
      /*
       * Update positional index of Kilobot entities
       */
      m_pcKilobotIndex->Update();
      /*
       * Delete obsolete adjacency matrices
       */
      for(TAdjacencyMatrix::iterator it = m_tCommMatrix.begin();
          it != m_tCommMatrix.end();
          ++it) {
         it->second.clear();
      }
      m_tTxNeighbors.clear();
      /*
       * Construct the adjacency matrix of transmitting robots
       */
      /* Buffer for the communicating entities */
      CSet<CKilobotCommunicationEntity*,SEntityComparator> cOtherKilobots;
      /* This map contains the pairs that have already been checked */
      unordered_map<size_t, std::pair<CKilobotCommunicationEntity*, CKilobotCommunicationEntity*> > mapPairsAlreadyChecked;
      /* Iterator for the above structure */
      unordered_map<size_t, std::pair<CKilobotCommunicationEntity*, CKilobotCommunicationEntity*> >::iterator itPair;
      /* Used as test key */
      std::pair<CKilobotCommunicationEntity*, CKilobotCommunicationEntity*> cTestKey;
      /* Used as hash for the test key */
      size_t unTestHash;
      /* The square distance between two Kilobots */
      Real fSqDistance;
      /* Go through the Kilobot entities */
      for(TAdjacencyMatrix::iterator it = m_tCommMatrix.begin();
          it != m_tCommMatrix.end();
          ++it) {
         /* Get a reference to the current Kilobot entity */
         CKilobotCommunicationEntity& cKilobot = *reinterpret_cast<CKilobotCommunicationEntity*>(GetSpace().GetEntityVector()[it->first]);
         /* Is this robot trying to transmit? */
         if(cKilobot.GetTxStatus() == CKilobotCommunicationEntity::TX_ATTEMPT) {
            /* Yes, add it to the list of transmitting robots */
            m_tTxNeighbors[cKilobot.GetIndex()];
            /* Get the list of Kilobots in range */
            cOtherKilobots.clear();
            m_pcKilobotIndex->GetEntitiesAt(cOtherKilobots, cKilobot.GetPosition());
            /* Go through the Kilobots in range */
            for(CSet<CKilobotCommunicationEntity*,SEntityComparator>::iterator it2 = cOtherKilobots.begin();
                it2 != cOtherKilobots.end();
                ++it2) {
               /* Get a reference to the neighboring Kilobot */
               CKilobotCommunicationEntity& cOtherKilobot = **it2;
               /* First, make sure the entities are not the same and
                  that they are both transmitting */
               if(&cKilobot != &cOtherKilobot &&
                  cOtherKilobot.GetTxStatus() == CKilobotCommunicationEntity::TX_ATTEMPT) {
                  /* Proceed if the pair has not been checked already */
                  if(&cKilobot < &cOtherKilobot) {
                     cTestKey.first = &cKilobot;
                     cTestKey.second = &cOtherKilobot;
                  }
                  else {
                     cTestKey.first = &cOtherKilobot;
                     cTestKey.second = &cKilobot;
                  }
                  unTestHash = HashKilobotPair(cTestKey);
                  itPair = mapPairsAlreadyChecked.find(unTestHash);
                  if(itPair == mapPairsAlreadyChecked.end() ||   /* Pair does not exist */
                     itPair->second.first != cTestKey.first ||   /* Pair exists, but first Kilobot involved is different */
                     itPair->second.second != cTestKey.second) { /* Pair exists, but second Kilobot involved is different */
                     /* Mark this pair as already checked */
                     mapPairsAlreadyChecked[unTestHash] = cTestKey;
                     /* Calculate square distance */
                     fSqDistance = SquareDistance(cKilobot.GetPosition(),
                                                  cOtherKilobot.GetPosition());
                     if(fSqDistance < Square(cOtherKilobot.GetTxRange())) {
                        /* cKilobot receives cOtherKilobot's message */
                        m_tTxNeighbors[cKilobot.GetIndex()].insert(&cOtherKilobot);
                     }
                     if(fSqDistance < Square(cKilobot.GetTxRange())) {
                        /* cOtherKilobot receives cKilobot's message */
                        m_tTxNeighbors[cOtherKilobot.GetIndex()].insert(&cKilobot);
                     }
                  } /* pair check */
               } /* entity identity + transmit check */
            } /* neighbors loop */
         } /* transmission check */
      } /* robot loop */
      /*
       * Go through transmitting robots and broadcast messages
       */
      /* Buffer to store the intersection data */
      SEmbodiedEntityIntersectionItem sIntersectionItem;
      /* Loop over transmitting robots */
      for(TAdjacencyMatrix::iterator it = m_tTxNeighbors.begin();
          it != m_tTxNeighbors.end();
          ++it) {
         /* Is this robot conflicting? */
         if(m_bIgnoreConflicts ||
            it->second.empty() ||
            m_pcRNG->Uniform(CRange<UInt32>(0, it->second.size() + 1)) == 0) {
            /* The robot can transmit */
            /* Get a reference to the current Kilobot entity */
            CKilobotCommunicationEntity& cKilobot = *reinterpret_cast<CKilobotCommunicationEntity*>(GetSpace().GetEntityVector()[it->first]);
            /* Change its transmission status */
            cKilobot.SetTxStatus(CKilobotCommunicationEntity::TX_SUCCESS);
            /* Go through its neighbors */
            cOtherKilobots.clear();
            m_pcKilobotIndex->GetEntitiesAt(cOtherKilobots, cKilobot.GetPosition());
            for(CSet<CKilobotCommunicationEntity*,SEntityComparator>::iterator it2 = cOtherKilobots.begin();
                it2 != cOtherKilobots.end();
                ++it2) {
               /* Get a reference to the neighboring Kilobot entity */
               CKilobotCommunicationEntity& cOtherKilobot = **it2;
               /* Make sure the robots are different */
               if(&cKilobot != &cOtherKilobot) {
                  /* Calculate distance */
                  fSqDistance = SquareDistance(cKilobot.GetPosition(),
                                               cOtherKilobot.GetPosition());
                  /* If robots are within transmission range and transmission succeeds... */
                  if(fSqDistance < Square(cKilobot.GetTxRange()) &&
                     m_pcRNG->Bernoulli(m_fRxProb)) {
                     /* cOtherKilobot receives cKilobot's message */
                     m_tCommMatrix[cOtherKilobot.GetIndex()].insert(&cKilobot);
                  }
               } /* identity check */
            } /* neighbor loop */
         } /* conflict check */
      } /* transmitters loop */
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::AddEntity(CKilobotCommunicationEntity& c_entity) {
      m_tCommMatrix.insert(
         std::make_pair<ssize_t, CSet<CKilobotCommunicationEntity*,SEntityComparator> >(
            c_entity.GetIndex(), CSet<CKilobotCommunicationEntity*,SEntityComparator>()));
      m_pcKilobotIndex->AddEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::RemoveEntity(CKilobotCommunicationEntity& c_entity) {
      m_pcKilobotIndex->RemoveEntity(c_entity);
      TAdjacencyMatrix::iterator it = m_tCommMatrix.find(c_entity.GetIndex());
      if(it != m_tCommMatrix.end())
         m_tCommMatrix.erase(it);
   }

   /****************************************/
   /****************************************/

   const CSet<CKilobotCommunicationEntity*,SEntityComparator>& CKilobotCommunicationMedium::GetKilobotsCommunicatingWith(CKilobotCommunicationEntity& c_entity) const {
      TAdjacencyMatrix::const_iterator it = m_tCommMatrix.find(c_entity.GetIndex());
      if(it != m_tCommMatrix.end()) {
         return it->second;
      }
      else {
         THROW_ARGOSEXCEPTION("Kilobot entity \"" << c_entity.GetId() << "\" is not managed by the Kilobot medium \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::SendOHCMessageTo(CKilobotEntity& c_robot,
                                                      message_t* pt_message) {
      /*
       * old   | new   | action
       * ------+-------+---------------------
       * ~null | ~null | delete old, copy new
       * ~null |  null | delete old, set null
       *  null | ~null | copy new
       *  null |  null | nothing to do
       */
      /* Get old message and delete it if necessary */
      message_t* ptOldMsg = GetOHCMessageFor(c_robot);
      if(ptOldMsg != NULL) delete ptOldMsg;
      /* Set message */
      m_mapOHCMessages[c_robot.GetIndex()] = NULL;
      if(pt_message != NULL)
         m_mapOHCMessages[c_robot.GetIndex()] = new message_t(*pt_message);
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationMedium::SendOHCMessageTo(std::vector<CKilobotEntity*>& vec_robots,
                                                      message_t* pt_message) {
      for(size_t i = 0; i < vec_robots.size(); ++i) {
         /*
          * old   | new   | action
          * ------+-------+---------------------
          * ~null | ~null | delete old, copy new
          * ~null |  null | delete old, set null
          *  null | ~null | copy new
          *  null |  null | nothing to do
          */
         /* Get old message and delete it if necessary */
         message_t* ptOldMsg = GetOHCMessageFor(*vec_robots[i]);
         if(ptOldMsg != NULL) delete ptOldMsg;
         /* Set message */
         m_mapOHCMessages[vec_robots[i]->GetIndex()] = NULL;
         if(pt_message != NULL)
            m_mapOHCMessages[vec_robots[i]->GetIndex()] = new message_t(*pt_message);
      }
   }

   /****************************************/
   /****************************************/

   message_t* CKilobotCommunicationMedium::GetOHCMessageFor(CKilobotEntity& c_robot) {
      /* Look for robot in map */
      unordered_map<ssize_t, message_t*>::iterator it = m_mapOHCMessages.find(c_robot.GetIndex());
      /* Return entry if robot found, NULL otherwise */
      return (it == m_mapOHCMessages.end()) ? NULL : (it->second);
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CKilobotCommunicationMedium,
                   "kilobot_communication",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "It simulates communication across Kilobot robots.",
                   "This medium is required to simulate communication across Kilobots. It works as\n"
                   "follows:\n"
                   "1. The medium calculates which robots can transmit at each time step. Every\n"
                   "   robot is assigned a non-transmission period. At the end of this period, the\n"
                   "   robot attempts transmission. It is successful with a probability that is\n"
                   "   inversely proportional to the number of transmitting robots in range. If\n"
                   "   successful, robot waits until the next period to transmit again. Otherwise\n"
                   "   it tries at the next time step.\n"
                   "2. It broadcasts the messages of the robots that can transmit. It is possible\n"
                   "   to specify the probability of message dropping by a robot as an optional\n"
                   "   parameter (see below).\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "<kilobot_communication id=\"kbc\" />\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to specify the probability with which a robot a drops an incoming\n"
                   "message. This is done by setting the attribute \"message_drop_prob\". When set\n"
                   "to 0, no message is ever dropped; when set to 1, every message is dropped.\n\n"
                   "<kilobot_communication id=\"kbc\" message_drop_prob=\"0.25\" />\n\n"
                   "It is also possible to ignore the effect of channel congestion. When two robots\n"
                   "are trying to send a message at the same time, a message conflict occurs. The\n"
                   "default behavior is to allow robots to complete message delivery according to a\n"
                   "random choice. If you don't want conflicts to be simulated, set the flag\n"
                   "'ignore_conflicts' to 'true':\n\n"
                   "<kilobot_communication id=\"kbc\" ignore_conflicts=\"true\" />\n"
                   ,
                   "Under development"
      );

   /****************************************/
   /****************************************/

}
