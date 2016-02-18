#ifndef IRRSIMULATION_H_INCLUDED
    #define IRRSIMULATION_H_INCLUDED

    #include <vector>
    #include <iostream>
    #include <string>
    #include <math.h>
    #include <map>
    #include <list>

    //#define USE_DEBUG_CONFIG
    #ifndef _IRR_STATIC_LIB_
        #define _IRR_STATIC_LIB_
    #endif // _IRR_STATIC_LIB_

    #include <irr/irrlicht.h>
    #include "btBulletCollisionCommon.h"
    #include "btBulletDynamicsCommon.h"
    #include "BulletCollision/CollisionShapes/btShapeHull.h"
    #include "LinearMath/btConvexHull.h"
    #include "LinearMath/btAlignedObjectArray.h"
    #include "LinearMath/btConvexHullComputer.h"

    #include <btBulletDynamicsCommon.h>
    #include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
    #include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
    #include "BulletSoftBody/btSoftBodyHelpers.h"
    #include "BulletSoftBody/btSoftBody.h"
    #include "BulletSoftBody/btDefaultSoftBodySolver.h"

    #include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
    #include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
    #include "LinearMath/btQuickprof.h"
    #include "LinearMath/btIDebugDraw.h"
    #include "LinearMath/btConvexHull.h"

#ifdef USE_MULTI_BODY_WORLD
    #include "BulletDynamics/Featherstone/btMultiBody.h"
    #include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
    #include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
    #include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
    #include "BulletDynamics/Featherstone/btMultiBodyLink.h"
    #include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
    #include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
    #include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#endif // USE_MULTI_BODY_WORLD

    //#include "BulletSoftBody/btSoftBody.h"
    #include "IrrDebugDraw.h"

#endif // IRRSIMULATION_H_INCLUDED
