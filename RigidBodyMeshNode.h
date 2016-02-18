#ifndef RIGIDBODYMESHNODE_H_INCLUDED
#define RIGIDBODYMESHNODE_H_INCLUDED
#include "irrSimulation.h"
#include "irr2bp.h"
#include "irrMotionState.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace physics;

class RigidBodyMeshNode:public irr2bp
{
public:
    enum OPTIMISATION
    {
        DEFAULT=0,//No optimisation
        TRICENTER=1,//Use face's triangle barycenter to make the shape
        BBOX=2,// Use BOUNDING BOX
        BSP_UNSUPPORTED_YET=3,//TO DO USE BOUNDING SPHERE
        BSO_UNSUPPORTED_YET=4// TO DO USE BOUNDING OVAL
    };
    static RigidBodyMeshNode* createConvexHullShapeNode(IMesh* mesh,const vector3df& position,
                                        const vector3df& rotation,const vector3df& scale,
                                        f32 mass,
                                        const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                                        ISceneManager* smgr,u16 optimisation=0);
    static RigidBodyMeshNode* createTriangleShapeNode(IMesh* mesh,const vector3df& position,
                                        const vector3df& rotation,const vector3df& scale,
                                        const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                                        ISceneManager* smgr);
    static RigidBodyMeshNode* createSphere(const vector3df& position,const f32& radius,u32 polyCount,
                                           const f32& mass,
                               const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                             ISceneManager* smgr);
    static RigidBodyMeshNode* createBox(const vector3df& boxSize,const vector3df& position,
                                        const vector3df& rotation,f32 mass,
                             const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                             ISceneManager* smgr);
    RigidBodyMeshNode(ISceneNode* node,btCollisionShape* shape,
                                     f32 mass,btSoftRigidDynamicsWorld* world);
//    RigidBodyMeshNode(IMesh* mesh,ISceneNode* node,btBroadphaseInterface* broadphase,
//                     btCollisionDispatcher* dispatcher,const btVector3& gravity,
//                     const btVector3& position,btSoftRigidDynamicsWorld* world);
    ~RigidBodyMeshNode();
    int drop(btSoftRigidDynamicsWorld* world,bool removeMesh=true);
    ISceneNode* getSceneNode();
    btRigidBody* getRigidBody();
    int setKinematic(bool value);
    bool isKinematic() const;
    bool isStatic() const;
    bool isStaticOrKinematic() const;
protected:
    ISceneNode* m_meshNode;
    btRigidBody* m_rigidBody;
};
#endif // RIGIDBODYMESHNODE_H_INCLUDED
