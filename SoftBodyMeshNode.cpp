#include "SoftBodyMeshNode.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace physics;

SoftBodyMeshNode* SoftBodyMeshNode::createSoftMeshNode(IMesh* mesh,const vector3df& position,
                                        const vector3df& rotation,const vector3df& scale,
                                        const core::stringw& texture,btBroadphaseInterface* broadphase,
                                         btCollisionDispatcher* dispatcher,const btVector3& gravity,
                                         btSoftRigidDynamicsWorld* world,
                                        ISceneManager* smgr)
{
    if((!mesh)||(!world)||(!smgr))return 0;
    IMeshSceneNode* meshNode = smgr->addMeshSceneNode(mesh);
    meshNode->setPosition(position);
    meshNode->setScale(scale);
    meshNode->setRotation(rotation);
    SoftBodyMeshNode* sf = new SoftBodyMeshNode(meshNode,broadphase,dispatcher,gravity,world);
    return (sf)?sf:0;
}

SoftBodyMeshNode::SoftBodyMeshNode(IMeshSceneNode* node,btBroadphaseInterface* broadphase,
                     btCollisionDispatcher* dispatcher,const btVector3& gravity,
                     btSoftRigidDynamicsWorld* world)
{
    m_meshNode = node;
    //node->setPosition(vector3df(position.getX(),position.getY(),position.getZ()));
//        dynamicsWorld = world;
    m_softBodyWorldInfo.m_broadphase = broadphase;
    m_softBodyWorldInfo.m_dispatcher = dispatcher;
    m_softBodyWorldInfo.m_sparsesdf.Initialize();
    m_softBodyWorldInfo.m_gravity = gravity;
    m_softBodyWorldInfo.air_density      =   (btScalar)1.2;
    m_softBodyWorldInfo.water_density   =   0;
    m_softBodyWorldInfo.water_offset   =   0;
    m_softBodyWorldInfo.water_normal   =   btVector3(0,0,0);

    setInternalVertices(world);
}

IMeshSceneNode* SoftBodyMeshNode::getMeshSceneNode()
{
    return m_meshNode;
}

btSoftBody* SoftBodyMeshNode::getSoftBody()
{
    return m_softBody;
}

void SoftBodyMeshNode::stepSimulation()
{
    for (int i=0; i<mb->getVertexCount(); i++)
    {
        int index = testMesh_map.find(i)->second;
        btSoftBody::Node* node = vertices.find(index)->second;
        mb_vertices[i].Pos.X = node->m_x.x();
        mb_vertices[i].Pos.Y = node->m_x.y();
        mb_vertices[i].Pos.Z = node->m_x.z();
    }
}

void SoftBodyMeshNode::setInternalVertices(btSoftRigidDynamicsWorld* dynamicsWorld)
{
    btVector3 position(getBtVector(m_meshNode->getPosition()));
    m_meshNode->setPosition(vector3df(0,0,0));
    // Indices
    std::vector<int> indices;
    // Temporary placeholder for nodes
    std::map<btSoftBody::Node*, int> node_map;
    u16* mb_indices;
    std::map<int, int> index_map;
    std::map<int, int> bullet_map;
    std::map<int, S3DVertex> vertex_map;
    MeshData testMesh;

    for (int cMeshBuffer=0; cMeshBuffer<m_meshNode->getMesh()->getMeshBufferCount(); cMeshBuffer++)
    {
        mb = m_meshNode->getMesh()->getMeshBuffer(cMeshBuffer);
        mb_vertices = (irr::video::S3DVertex*)mb->getVertices();
        mb_indices  = mb->getIndices();
    }

    int count = 0;

    for (int i=0; i<mb->getIndexCount(); i++)
    {
        int iIndex = mb_indices[i];
        vector3df iVertex = mb_vertices[iIndex].Pos;
        bool isFirst = true;
        for (int j=0; j<i; j++)
        {
            int jIndex = mb_indices[j];
            vector3df jVertex = mb_vertices[jIndex].Pos;
            if (iVertex == jVertex)
            {
                index_map.insert(std::make_pair(i, j));
                isFirst = false;
                break;
            }
        }

        if (isFirst)
        {

            index_map.insert(std::make_pair(i, i));

            bullet_map.insert(std::make_pair(i, count));

            vertex_map.insert(std::make_pair(count, mb_vertices[iIndex]));
            count++;
        }
    }

    testMesh.indexCount = mb->getIndexCount();
    testMesh.mqo_indices = new int[testMesh.indexCount];
    testMesh.vertexCount = vertex_map.size();
    testMesh.mqo_vertices = new btScalar[testMesh.vertexCount*3];
//
//        std::cout << "IndexCount=" << testMesh.indexCount << ", VertexCount=" << testMesh.vertexCount << std::endl;

    for (int j=0; j<mb->getIndexCount(); j++)
    {
        int index1 = index_map.find(j)->second;

        int index2 = bullet_map.find(index1)->second;
        testMesh.mqo_indices[j]   = index2;
    }

    for (int j=0; j<testMesh.vertexCount; j++)
    {
        testMesh.mqo_vertices[3*j] =   vertex_map[j].Pos.X;
        testMesh.mqo_vertices[3*j+1] = vertex_map[j].Pos.Y;
        testMesh.mqo_vertices[3*j+2] = -vertex_map[j].Pos.Z;
    }

    m_softBody = btSoftBodyHelpers::CreateFromTriMesh(
    m_softBodyWorldInfo, testMesh.mqo_vertices, testMesh.mqo_indices, testMesh.indexCount/3);

    for (int i=0; i<m_softBody->m_faces.size(); i++)
    {
        btSoftBody::Face face = m_softBody->m_faces[i];

        for (int j=0; j<3; j++)
        {
            if (node_map.find(face.m_n[j]) == node_map.end())
            {
                node_map.insert(std::make_pair(face.m_n[j], node_map.size()));
            }
        }

        for (int j=0; j<3; j++)
        {
            indices.push_back(node_map.find(face.m_n[j])->second);
        }
    }
    // Reverse node->index to index->node (should be unique on both sides)
    std::map<btSoftBody::Node*, int>::const_iterator node_iter;
    for (node_iter = node_map.begin(); node_iter != node_map.end(); ++node_iter)
    {
        vertices.insert(std::make_pair(node_iter->second, node_iter->first));
    }

    std::map<int, btSoftBody::Node*>::const_iterator it;
    for (int i=0; i<mb->getVertexCount(); i++)
    {
        for (it=vertices.begin(); it != vertices.end(); ++it)
        {
            int v_index = it->first;
            btSoftBody::Node* node = it->second;
            if (node->m_x.x() ==  mb_vertices[i].Pos.X &&
                node->m_x.y() ==  mb_vertices[i].Pos.Y &&
                node->m_x.z() == -mb_vertices[i].Pos.Z)
            {
                testMesh_map.insert(std::make_pair(i, v_index));
                break;
            }
        }
    }

    btSoftBody::Material*	pm=m_softBody->appendMaterial();
    m_softBody->m_cfg.kDP = 0.0;// Damping coefficient [0,1]
    m_softBody->m_cfg.kDF = 0.8;// Dynamic friction coefficient [0,1]
    m_softBody->m_cfg.kMT = 0.02;// Pose matching coefficient [0,1]
    m_softBody->m_cfg.kCHR = 0.2;// Rigid contacts hardness [0,1]
    m_softBody->m_cfg.kKHR = 0.8;// Kinetic contacts hardness [0,1]
    m_softBody->m_cfg.kSHR = 1.0;// Soft contacts hardness [0,1]
    m_softBody->m_cfg.piterations=2;
    m_softBody->m_cfg.collisions=btSoftBody::fCollision::CL_SS+btSoftBody::fCollision::CL_RS;
    m_softBody->m_materials[0]->m_kLST = 1;
    m_softBody->m_materials[0]->m_kAST = 0.8;
    m_softBody->m_materials[0]->m_kVST = 0.8;
//        m_softBody->scale(btVector3(2,2,2));
    /**SCale must be set before next lines! otherwise you'll get unwated shape behaviour*/
    m_softBody->setPose(true, true);
    m_softBody->generateBendingConstraints(2,pm);
    m_softBody->generateClusters(2);
    m_softBody->randomizeConstraints();

    btMatrix3x3 m;
    m.setIdentity();
    m_softBody->transform(btTransform(m,position));
    dynamicsWorld->addSoftBody(m_softBody);
}

int SoftBodyMeshNode::drop(btSoftRigidDynamicsWorld* world,bool removeMesh)
{
    if(m_softBody)
    {
        if(removeMesh)
        {
            if(static_cast<ISceneNode*>(m_softBody->getUserPointer()))
            static_cast<ISceneNode*>(m_softBody->getUserPointer())->remove();
        }

		world->removeCollisionObject(m_softBody);
		return 0;
    }
    return -1;
}
