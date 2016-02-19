#ifndef GAMESOFTOBJECT_H_INCLUDED
#define GAMESOFTOBJECT_H_INCLUDED
#include "SoftBodyMeshNode.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;

namespace gameEngine
{
    class gameSoftObject:public SoftBodyMeshNode
    {
    public:
        gameSoftObject(IMeshSceneNode* node,btBroadphaseInterface* broadphase,
                     btCollisionDispatcher* dispatcher,const btVector3& gravity,
                     btSoftRigidDynamicsWorld* world);
        ~gameSoftObject();
    };
}

#endif // GAMESOFTOBJECT_H_INCLUDED
