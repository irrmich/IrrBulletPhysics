#include "gameSoftObject.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gameEngine;

gameSoftObject::gameSoftObject(IMeshSceneNode* node,btBroadphaseInterface* broadphase,
                     btCollisionDispatcher* dispatcher,const btVector3& gravity,
                     btSoftRigidDynamicsWorld* world):SoftBodyMeshNode(node,broadphase,dispatcher,gravity,world)
{

}

gameSoftObject::~gameSoftObject()
{

}
