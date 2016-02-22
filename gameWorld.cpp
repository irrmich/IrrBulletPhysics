#include "gameWorld.h"
using namespace gameEngine;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

gameWorld::gameWorld()
{

}

void gameWorld::preRender()
{

}

void gameWorld::drop()
{
    for(std::vector<gameSimpleCar*>::iterator it=m_cars.begin();it!=m_cars.end();it++)
    {
        delete (*it);
    }
    m_cars.clear();
    for(std::vector<RigidBodyMeshNode*>::iterator it=m_rigidObjects.begin();it!=m_rigidObjects.end();it++)
    {
        (*it)->drop(static_cast<btSoftRigidDynamicsWorld*>(m_phyWorld.m_world));
        delete (*it);
    }
    m_rigidObjects.clear();
    for(std::vector<SoftBodyMeshNode*>::iterator it=m_softObjects.begin();it!=m_softObjects.end();it++)
    {
        (*it)->drop(static_cast<btSoftRigidDynamicsWorld*>(m_phyWorld.m_world));
        delete (*it);
    }
    m_softObjects.clear();
}

btDynamicsWorld* gameWorld::getPhysicsWorld()
{
    return m_phyWorld.m_world;
}

void gameWorld::runPhysics(bool fixed,float fps,int maxsubSteps)
{
    if(fixed)
    {
        m_phyWorld.m_world->stepSimulation(1.f/fps,maxsubSteps);
    } else
    {
        m_phyWorld.deltaTime = m_dev->getTimer()->getTime()-m_phyWorld.currentTime;
        m_phyWorld.currentTime = m_dev->getTimer()->getTime();
        m_phyWorld.m_world->stepSimulation(m_phyWorld.deltaTime/1000.f,maxsubSteps);
    }

//    if(m_checkContactManifolds)
//        this->onCollision();
#ifdef USE_MULTI_BODY_WORLD
    updateMultiBodies();
#endif // USE_MULTI_BODY_WORLD
}

#ifdef USE_MULTI_BODY_WORLD
void gameWorld::updateMultiBodies()
{
	const int	numObjects=m_world->getNumCollisionObjects();
	for(int i=0;i<numObjects;i++)
	{
		btCollisionObject* colObj=m_world->getCollisionObjectArray()[i];
		btMultiBodyLinkCollider* body=btMultiBodyLinkCollider::upcast(colObj);
		if(!body)continue;
//		if(body->getMotionState())
//		{
//			continue;
//		}
		else
		{
			btTransform trans=colObj->getWorldTransform();
			IMeshSceneNode* node=static_cast<IMeshSceneNode*>(body->getUserPointer());
			vector3df pos,rot;
			irr2bp::getIrrTransform(pos,rot,trans);
			node->setPosition(pos);
			node->setRotation(rot);
		}
	}
}
#endif // USE_MULTI_BODY_WORLD

int gameWorld::initPhysics()
{
    m_phyWorld.broadPhase = new btDbvtBroadphase;
    m_phyWorld.collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration;
    m_phyWorld.collisionDispatcher = new btCollisionDispatcher(m_phyWorld.collisionConfiguration);
    #ifdef USE_MULTI_BODY_WORLD
        m_phyWorld.constraintSolver = new btMultiBodyConstraintSolver;
        m_phyWorld.m_world = new btMultiBodyDynamicsWorld(m_phyWorld.collisionDispatcher,
                                                          m_phyWorld.broadPhase,
                                                          static_cast<btMultiBodyConstraintSolver*>(m_phyWorld.constraintSolver),
                                                          m_phyWorld.collisionConfiguration);
    #else
        m_phyWorld.constraintSolver = new btSequentialImpulseConstraintSolver;
        m_phyWorld.softBSolv = new btDefaultSoftBodySolver;
        m_phyWorld.m_world = new btSoftRigidDynamicsWorld(m_phyWorld.collisionDispatcher,
                                                          m_phyWorld.broadPhase,
                                                          m_phyWorld.constraintSolver,
                                                          m_phyWorld.collisionConfiguration,
                                                          m_phyWorld.softBSolv);
    #endif // USE_MULTI_BODY_WORLD
    m_phyWorld.currentTime = m_dev->getTimer()->getTime();
}

bool gameWorld::init(E_DRIVER_TYPE driverType,u32 windowWidth,u32 windowHeight,u32 pixelResolution,bool fullScreen,
                  bool stencilBuffer,bool vsync)
{
    SIrrlichtCreationParameters irrParam;
    irrParam.AntiAlias = 2;
    irrParam.Bits = pixelResolution;
    irrParam.DeviceType = EIDT_BEST;
    irrParam.DriverType = driverType;
    irrParam.Fullscreen = fullScreen;
    irrParam.Stencilbuffer = stencilBuffer;
    irrParam.Vsync = vsync;
    irrParam.WindowSize = dimension2du(windowWidth,windowHeight);

    /*m_dev = createDevice(driverType, dimension2du(windowWidth,windowHeight), pixelResolution,
            fullScreen,stencilBuffer,vsync);*/
    m_dev = createDeviceEx(irrParam);

    m_dev->getCursorControl()->setVisible(false);
    m_drv = m_dev->getVideoDriver();
    //For better texture quality
    m_drv->setTextureCreationFlag(ETCF_OPTIMIZED_FOR_QUALITY,true);
    //ATI DRIVERS SUCKS if you don't disable ETCF_CREATE_MIP_MAPS
    m_drv->setTextureCreationFlag(ETCF_CREATE_MIP_MAPS,false);
    m_smgr = m_dev->getSceneManager();

//    m_univ = new irr::physics::universe(m_dev,vector3df(0,-9.8,0));
    //m_evTracker = new gameEventTracker();
    m_dev->setEventReceiver(this);

    m_debDraw = new irrDebugDraw(m_drv,m_smgr);

    setDefaultCamera();
    setSkyBoxandDom(mediaDir+"irrlicht2_up.jpg",
                          mediaDir+"irrlicht2_dn.jpg",
                          mediaDir+"irrlicht2_lf.jpg",
                          mediaDir+"irrlicht2_rt.jpg",
                          mediaDir+"irrlicht2_ft.jpg",
                          mediaDir+"irrlicht2_bk.jpg");
    initPhysics();
    return true;
}

void gameWorld::setDefaultCamera()
{
    m_cam = m_smgr->addCameraSceneNodeFPS(0,100.f,0.06f);
    m_cam->setPosition(vector3df(10,5,10));
    m_cam->setTarget(vector3df(0,0,0));
}

void gameWorld::setSkyBoxandDom(const stringw& textureUp,const stringw& textureDn,
                                 const stringw& textureLt,const stringw& textureRt,
                                 const stringw& textureFd,const stringw& textureBk)
{
    m_smgr->addSkyBoxSceneNode(
    m_drv->getTexture(textureUp),
    m_drv->getTexture(textureDn),
    m_drv->getTexture(textureLt),
    m_drv->getTexture(textureRt),
    m_drv->getTexture(textureFd),
    m_drv->getTexture(textureBk));
}

void gameWorld::updateSoftBodies()
{
    for(std::vector<SoftBodyMeshNode*>::iterator it=m_softObjects.begin();it!=m_softObjects.end();it++)
    {
        (*it)->stepSimulation();
    }
}

bool gameWorld::runGameLoop()
{
    while(m_dev->run())
    {
        runPhysics(true,60,1);
        updateVehicles();
        updateSoftBodies();
        gameLoop();
    }
}

bool gameWorld::gameLoop()
{
    m_drv->beginScene(true, true, SColor(0,255,255,255));
    //You can avoid the drawAxes (then delete IrrDebugDraw from project) no probelm
    m_debDraw->drawAxes(btVector3(0,255,0),btVector3(0,0,255),btVector3(255,0,0),2000);

    //btVector3 velocity = car->m_vehicle->getRigidBody()->getLinearVelocity();
    //coutVec(vector3df(velocity.getX(),velocity.getY(),velocity.getZ()),"velocity = ");
    //You could change the parameters here if your device or computer are slow
//    m_univ->runPhysics();
    //you are not obliged to execute the camFollowVehicle but it's cool with it
    //camFollowVehicle(car->getPosition(),cam);
    m_smgr->drawAll();
    m_drv->endScene();
}

void gameWorld::updateVehicles()
{
    for(std::vector<gameSimpleCar*>::iterator it=m_cars.begin();it!=m_cars.end();it++)
    {
//        (*it)->setCameraFollower(m_cam);
        if(!(*it)->isReady())continue;
        (*it)->moveVehicle();
        (*it)->restoreVehicleSteering();
    }
}

bool gameWorld::OnEvent(const SEvent &event)
{
    for(u32 i=0;i<m_cars.size();i++)
    {
        if(m_cars[i]->isReady())
        {
            m_cars[i]->OnEvent(event);
        }
    }
}

gameWorld::~gameWorld()
{
    drop();
//    if(m_univ)delete m_univ;
    if(m_debDraw)delete m_debDraw;
    if(m_dev)m_dev->drop();
}
