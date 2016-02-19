#ifndef IRRMOTIONSTATE_H_INCLUDED
#define IRRMOTIONSTATE_H_INCLUDED

#include "irrSimulation.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

class irrMotionState:public btMotionState{
public:
    irrMotionState(ISceneNode* irrNode);

    virtual ~irrMotionState();
    virtual void getWorldTransform(btTransform &worldTrans) const;
    virtual void setWorldTransform(const btTransform &worldTrans);
    virtual bool setSceneNode(ISceneNode* irrNode,bool removePreviousNode=true);

protected:
    btTransform m_worldTransform;
    ISceneNode* m_irrNode;
};

/*class CMotionState : public btDefaultMotionState
{
    public:
		CMotionState(btRigidBody * body,const btTransform &startTrans=btTransform::getIdentity(),
                const btTransform &centerOfMassOffset=btTransform::getIdentity());

        virtual ~CMotionState();

		virtual void getWorldTransform(btTransform &worldTrans);
        virtual void setWorldTransform(const btTransform &worldTrans);

    protected:
		btRigidBody * m_body;
		ISceneNode * m_irrNode;
};*/

#endif // IRRMOTIONSTATE_H_INCLUDED
