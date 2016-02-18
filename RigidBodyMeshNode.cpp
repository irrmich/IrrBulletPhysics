#include "RigidBodyMeshNode.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace physics;

RigidBodyMeshNode* RigidBodyMeshNode::createConvexHullShapeNode(IMesh* mesh,const vector3df& position,
                                        const vector3df& rotation,const vector3df& scale,
                                        f32 mass,
                                        const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                                        ISceneManager* smgr,u16 optimisation)
{
    if((!mesh)||(!smgr)||(!world))return 0;
    IMeshSceneNode* meshNode = smgr->addMeshSceneNode(mesh);
    if(!meshNode)return 0;
    meshNode->setPosition(position);
    meshNode->setScale(scale);
    meshNode->setRotation(rotation);

    if(texture!="")
    {
        meshNode->setMaterialTexture(0,smgr->getVideoDriver()->getTexture(texture.c_str()));
    }

    btCollisionShape* shape;
    switch(optimisation)
    {
    case DEFAULT:
        {
            shape = createConvexHullShape(mesh,scale);
        }break;
    case TRICENTER:
        {
            shape = createConvexHullShapeTM(mesh,scale);
        }break;
    case BBOX:
        {
            shape = createConvexHullShapeBB(meshNode->getTransformedBoundingBox(),mesh,scale);
        }break;
    case BSP_UNSUPPORTED_YET:
        {
            std::cout << "OPTIMISATION::BSP in TODO list" << std::endl;
            shape = 0;
        }
    case BSO_UNSUPPORTED_YET:
        {//TODO
            std::cout << "OPTIMISATION::BSO in TODO list" << std::endl;
            shape = 0;
        }break;
    default:
        {
            shape = createConvexHullShape(mesh,scale);
        }break;
    }
    if(!shape)return 0;
    RigidBodyMeshNode* rb = new RigidBodyMeshNode(meshNode,shape,mass,world);
    return (rb)?rb:0;
}

RigidBodyMeshNode* RigidBodyMeshNode::createTriangleShapeNode(IMesh* mesh,const vector3df& position,
                                        const vector3df& rotation,const vector3df& scale,
                                        const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                                        ISceneManager* smgr)
{
    if((!mesh)||(!smgr)||(!world))return 0;
    IMeshSceneNode* meshNode = smgr->addMeshSceneNode(mesh);
    if(!meshNode)return 0;
    meshNode->setPosition(position);
    meshNode->setScale(scale);
    meshNode->setRotation(rotation);

    if(texture!="")
    {
        meshNode->setMaterialTexture(0,smgr->getVideoDriver()->getTexture(texture.c_str()));
    }

    btCollisionShape* shape = new btBvhTriangleMeshShape(createTriangleMesh(mesh,scale),false,true);
    RigidBodyMeshNode* rb = new RigidBodyMeshNode(meshNode,shape,0,world);
    return (rb)?rb:0;
}

RigidBodyMeshNode* RigidBodyMeshNode::createSphere(const vector3df& position,const f32& radius,
                                                   u32 polyCount,const f32& mass,
                               const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                             ISceneManager* smgr)
{
    if((polyCount<=0)||(radius<=0)||(mass<0)||(!smgr)||(!world))return 0;

    btVector3 btBodyPos(position.X,position.Y,position.Z);
    ISceneNode* sphereNode = smgr->addSphereSceneNode(radius,polyCount);
    sphereNode->setPosition(position);

    if(texture!="")
    {
        sphereNode->setMaterialTexture(0,smgr->getVideoDriver()->getTexture(texture.c_str()));
    }

    btCollisionShape* shape = new btSphereShape(radius);

    RigidBodyMeshNode* rb = new RigidBodyMeshNode(sphereNode,shape,mass,world);
    return (rb)?rb:0;
}

RigidBodyMeshNode* RigidBodyMeshNode::createBox(const vector3df& boxSize,const vector3df& position,
                                        const vector3df& rotation,f32 mass,
                                         const core::stringw& texture,btSoftRigidDynamicsWorld* world,
                                         ISceneManager* smgr)
{
    if((mass<0)||(!smgr)||(!world))return 0;
    ISceneNode* cubeNode = smgr->addCubeSceneNode(1.f);
    cubeNode->setScale(boxSize);
    cubeNode->setPosition(position);
    cubeNode->setRotation(rotation);

    if(texture!="")
    {
        cubeNode->setMaterialTexture(0,smgr->getVideoDriver()->getTexture(texture.c_str()));
    }
    btCollisionShape* shape = new btBoxShape(btVector3(boxSize.X*0.5,boxSize.Y*0.5,boxSize.Z*0.5));

    RigidBodyMeshNode* rb = new RigidBodyMeshNode(cubeNode,shape,mass,world);
    return (rb)?rb:0;
}

RigidBodyMeshNode::RigidBodyMeshNode(ISceneNode* node,btCollisionShape* shape,
                                     f32 mass,btSoftRigidDynamicsWorld* world)
{
    m_meshNode=node;
    vector3df position(node->getPosition());
    vector3df rotation(DEGTORAD64*node->getRotation());
    core::quaternion quat;
    quat.set(rotation.X,rotation.Y,rotation.Z);

    btTransform transf;
    transf.setIdentity();
    transf.setOrigin(btVector3(position.X,position.Y,position.Z));
    transf.setRotation(btQuaternion(quat.X,quat.Y,quat.Z,quat.W));

    btVector3 localInertia;
//    if(mass)
        shape->calculateLocalInertia(mass,localInertia);

    irrMotionState *usrMotionState = new irrMotionState(node);
    usrMotionState->setWorldTransform(transf);

    //btMotionState* motionS = new btDefaultMotionState(transf);
    m_rigidBody = new btRigidBody(mass,usrMotionState,shape,localInertia);

    m_rigidBody->setUserPointer((void*)(node));
    world->addRigidBody(m_rigidBody);
}

//RigidBodyMeshNode::RigidBodyMeshNode(IMesh* mesh,ISceneNode* node,btBroadphaseInterface* broadphase,
//                     btCollisionDispatcher* dispatcher,const btVector3& gravity,
//                     const btVector3& position,btSoftRigidDynamicsWorld* world)
//{
//
//}

ISceneNode* RigidBodyMeshNode::getSceneNode()
{
    return m_meshNode;
}

btRigidBody* RigidBodyMeshNode::getRigidBody()
{
    return m_rigidBody;
}

int RigidBodyMeshNode::setKinematic(bool value)
{
//Positive mass object can't be kinematic (static movable object)
    if(!m_rigidBody->isStaticOrKinematicObject())return -1;
    if(value==true)
    {
        m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        m_rigidBody->setActivationState(DISABLE_DEACTIVATION);
    } else
    {
        m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
        m_rigidBody->setActivationState(WANTS_DEACTIVATION);
    }
}

bool RigidBodyMeshNode::isKinematic() const
{
    return m_rigidBody->isKinematicObject();
}

bool RigidBodyMeshNode::isStatic() const
{
    return m_rigidBody->isStaticObject();
}

bool RigidBodyMeshNode::isStaticOrKinematic() const
{
    return m_rigidBody->isStaticOrKinematicObject();
}

int RigidBodyMeshNode::drop(btSoftRigidDynamicsWorld* world,bool removeMesh)
{
    if(m_rigidBody)
    {
        if(removeMesh)
        {
            if(static_cast<ISceneNode*>(m_rigidBody->getUserPointer()))
            static_cast<ISceneNode*>(m_rigidBody->getUserPointer())->remove();
        }

        delete m_rigidBody->getMotionState();
		world->removeCollisionObject(m_rigidBody);
		return 0;
    }
    return -1;
}

RigidBodyMeshNode::~RigidBodyMeshNode()
{

}
