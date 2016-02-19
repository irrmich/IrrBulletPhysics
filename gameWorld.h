#ifndef GAMEWORLD_H_INCLUDED
#define GAMEWORLD_H_INCLUDED
//#include "irrPhysics.h"
#include "irrSimulation.h"
//#include "irrSong.h"
//#include "irrNetwork.h"
#include <irr/IEventReceiver.h>
//#include "gameStaticObject.h"
//#include "gameKinematicObject.h"
//#include "gameDynamicObject.h"
#include "SoftBodyMeshNode.h"
#include "RigidBodyMeshNode.h"
#include "gameSimpleCar.h"
//#include "gameStaticLight.h"
//#include "gameKinematicLight.h"
//#include "gameDynamicLight.h"
#include "gameRigidObject.h"
#include "gameSoftObject.h"
#include "gameTree.h"
#include "gameHuman.h"
#include "gameHouse.h"
#include "gameMoto.h"
#include "gamePlane.h"
//#include "gameDatabase.h"

#ifdef USE_MULTI_BODY_WORLD
struct btMultiBodySettings
{
	btMultiBodySettings()
	{
		m_numLinks = 0;
		m_constaintLimit = 1;
		m_basePosition.setZero();
		m_extremityPosition.setZero();
		m_isFixedBase = true;
		m_isFixedExtremity = true;
		m_usePrismatic = false;
		m_canSleep = true;
		m_createConstraints = false;
		m_disableParentCollision = false;
		m_rigidify = false;
		m_makeSouple = false;
	}
	float       m_constaintLimit;
	int			m_numLinks;
	btVector3	m_basePosition;
	btVector3   m_extremityPosition;
	bool        m_rigidify;
	bool        m_makeSouple;
	bool		m_isFixedBase;
	bool        m_isFixedExtremity;
	bool		m_usePrismatic;
	bool		m_canSleep;
	bool		m_createConstraints;
	bool		m_disableParentCollision;
};
#endif // USE_MULTI_BODY_WORLD

using namespace gameEngine;

namespace gameEngine
{
class gameWorld:public IEventReceiver
    {
    public:
        struct PhysicsWorld
        {
            u32 currentTime;
            u32 deltaTime;
            btBroadphaseInterface* broadPhase;
            btDefaultCollisionConfiguration* collisionConfiguration;
            btCollisionDispatcher* collisionDispatcher;
    //        btMultiBodyConstraintSolver* constraintSolver;
            btConstraintSolver* constraintSolver;
            btSoftBodySolver* softBSolv;
    //        btMultiBodyDynamicsWorld* m_world;
            btDynamicsWorld* m_world;
            u32 frequence;
        };

        gameWorld();
        ~gameWorld();
        btDynamicsWorld* getPhysicsWorld();
        void runPhysics(bool fixed=true,float fps=60,int maxsubSteps=1);
        virtual bool createGameWorld()=0;
        virtual bool init(irr::video::E_DRIVER_TYPE driverType,irr::u32 windowWidth,
                  irr::u32 windowHeight,irr::u32 pixelResolution,
                  bool fullScreen,bool stencilBuffer,bool vsync);
        virtual void setDefaultCamera();
        virtual bool runGameLoop();
        virtual bool gameLoop();
        virtual void preRender();
        virtual bool OnEvent(const SEvent &event);
        void setSkyBoxandDom(const irr::core::stringw& textureUp,const irr::core::stringw& textureDn,
                         const irr::core::stringw& textureLt,const irr::core::stringw& textureRt,
                         const irr::core::stringw& textureFd,const irr::core::stringw& textureBk);

    protected:
        #ifdef USE_MULTI_BODY_WORLD
        void updateMultiBodies();
        #endif
        void updateSoftBodies();
        void updateVehicles();
        void drop();
        /**Timing for stepping*/
        int initPhysics();
        PhysicsWorld m_phyWorld;

        irr::IrrlichtDevice* m_dev;
        irr::scene::ISceneManager* m_smgr;
        irr::video::IVideoDriver* m_drv;
        irr::scene::ICameraSceneNode* m_cam;
        //gameEventTracker* m_evTracker;
        irrDebugDraw* m_debDraw;
//        irr::physics::universe* m_univ;

        irr::core::stringw mediaDir = "media/";
        irr::core::stringw modelDir = "models/";

//        std::vector<gameStaticObject*> m_staticObjs;
//        std::vector<gameDynamicObject*> m_dynamicObjs;
//        std::vector<gameKinematicObject*> m_kinematicObjs;
        std::vector<RigidBodyMeshNode*> m_rigidObjects;
        std::vector<SoftBodyMeshNode*> m_softObjects;
        std::vector<gameHuman*> m_humans;
        std::vector<gameTree*> m_trees;
        std::vector<gameHouse*> m_houses;
        std::vector<gameSimpleCar*> m_cars;
        std::vector<gameMoto*> m_motos;
        std::vector<gamePlane*> m_planes;
//        std::vector<gameStaticLight*> m_staticLights;
//        std::vector<gameKinematicLight*> m_kinematicLights;
    };
}


#endif // GAMEWORLD_H_INCLUDED
