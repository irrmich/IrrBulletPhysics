#include "gameRigidObject.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gameEngine;

gameRigidObject::gameRigidObject(ISceneNode* node,btCollisionShape* shape,
                                f32 mass,btSoftRigidDynamicsWorld* world):RigidBodyMeshNode(node,shape,mass,world)
{

}

gameRigidObject::~gameRigidObject()
{

}
