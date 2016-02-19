#ifndef SOFTBODYMESHNODE_H_INCLUDED
#define SOFTBODYMESHNODE_H_INCLUDED
#include "irrSimulation.h"
#include "irr2bp.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace physics;

class SoftBodyMeshNode:public irr2bp
{
    struct MeshData
    {
        btScalar *mqo_vertices;
        int *mqo_indices;
        int indexCount, vertexCount;
    };
public:
    static SoftBodyMeshNode* createSoftMeshNode(IMesh* mesh,const vector3df& position,
                                        const vector3df& rotation,const vector3df& scale,
                                        const core::stringw& texture,btBroadphaseInterface* broadphase,
                                         btCollisionDispatcher* dispatcher,const btVector3& gravity,
                                         btSoftRigidDynamicsWorld* world,
                                        ISceneManager* smgr);
    SoftBodyMeshNode(IMeshSceneNode* node,btBroadphaseInterface* broadphase,
                     btCollisionDispatcher* dispatcher,const btVector3& gravity,
                     btSoftRigidDynamicsWorld* world);
    IMeshSceneNode* getMeshSceneNode();
    btSoftBody* getSoftBody();
    void stepSimulation();
    int drop(btSoftRigidDynamicsWorld* world,bool removeMesh=true);
protected:
    void setInternalVertices(btSoftRigidDynamicsWorld* dynamicsWorld);
    IMeshSceneNode* m_meshNode;
//    int count = 0;
    btSoftBodyWorldInfo   m_softBodyWorldInfo;
    btSoftBody* m_softBody;
    IMeshBuffer *mb;
    video::S3DVertex* mb_vertices;
    // Vertices
    std::map<int, btSoftBody::Node*> vertices;
    std::map<int, int> testMesh_map;

//    btSoftRigidDynamicsWorld* dynamicsWorld;
};

#endif // SOFTBODYMESHNODE_H_INCLUDED
