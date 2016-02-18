#ifndef GAMERIGIDOBJECT_H_INCLUDED
#define GAMERIGIDOBJECT_H_INCLUDED
#include "RigidBodyMeshNode.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;

namespace gameEngine
{
    class gameRigidObject:public RigidBodyMeshNode
    {
    public:
        gameRigidObject(ISceneNode* node,btCollisionShape* shape,
                        f32 mass,btSoftRigidDynamicsWorld* world);
        ~gameRigidObject();
    };
}

#endif // GAMERIGIDOBJECT_H_INCLUDED
