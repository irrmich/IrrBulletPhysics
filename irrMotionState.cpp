#include "irrMotionState.h"

irrMotionState::irrMotionState(ISceneNode* irrNode)
{
    m_irrNode = irrNode;
}

irrMotionState::~irrMotionState(){}

void irrMotionState::getWorldTransform(btTransform &worldTrans) const
{
     worldTrans = m_worldTransform;
}

void irrMotionState::setWorldTransform(const btTransform &worldTrans)
{
    m_worldTransform = worldTrans;
    btQuaternion orientation = m_worldTransform.getRotation();
    btVector3 position = m_worldTransform.getOrigin();
    quaternion quat(orientation.getX(),orientation.getY(),orientation.getZ(),orientation.getW());
    vector3df eulerAngles;

    quat.toEuler(eulerAngles);
    m_irrNode->setPosition(vector3df(position.x(),position.y(),position.z()));
    m_irrNode->setRotation(RADTODEG*eulerAngles);
}

bool irrMotionState::setSceneNode(ISceneNode* irrNode,bool removePreviousNode)
{
    if(removePreviousNode)
    {
        m_irrNode->remove();
    }
    m_irrNode = irrNode;
}

/*CMotionState::CMotionState(btRigidBody * body,const btTransform &startTrans, const btTransform &centerOfMassOffset)
{
	m_startWorldTrans = startTrans;
	m_graphicsWorldTrans = startTrans;
	m_centerOfMassOffset = centerOfMassOffset;
	m_body = body;
	m_irrNode = static_cast<ISceneNode*>(m_body->getUserPointer());
	if(!m_irrNode)
		{
		    std::cout << "Error during motion state creation. No irrlicht node engaged" << std::endl;
		}
}

void CMotionState::getWorldTransform(btTransform &worldTrans)
{
	worldTrans = 	m_centerOfMassOffset.inverse() * m_graphicsWorldTrans ;

}
void CMotionState::setWorldTransform(const btTransform &worldTrans)
{
	if(m_irrNode)
	{
		m_irrNode->setPosition(vector3df(worldTrans.getOrigin().getX(),worldTrans.getOrigin().getY(),
                                   worldTrans.getOrigin().getZ()));
		//Seems to be more precise

		irr::core::matrix4 mat;
		irr::f32 * ptr;
		ptr = mat.pointer();
		worldTrans.getOpenGLMatrix(ptr);

		m_irrNode->setRotation(mat.getRotationDegrees());
		//m_irrNode->setRotation(QuaternionToIrrEuler(worldTrans.getRotation()));
		m_graphicsWorldTrans = worldTrans * m_centerOfMassOffset ;

	}
	else
	{
		std::cout << "Error during motion state updating." << std::endl;;
	}

}

CMotionState::~CMotionState()
{
}*/
