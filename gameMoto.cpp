#include "gameMoto.h"
#define USE_OPTIMIZED_CODE
using namespace gameEngine;
using namespace physics;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

gameMoto::gameMoto(btDynamicsWorld* world,ISceneManager* smgr,
                              const vector3df& position,
                              const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile):/*m_univ(univ),*/
VehicleRaycastMeshNode(world,smgr,position,rotation,wheelMeshFile,wheelTextureFile,chassisTextureFile)
{
//    m_univ->addVehicleRaycastMeshNode(m_id,position,rotation,static_cast<VehicleRaycastMeshNode*>(this));
    useLight=true;
    m_isReady = init(position,rotation,wheelMeshFile,wheelTextureFile,chassisTextureFile);
}

int gameMoto::mountVehiclePieces()
{
    if(m_carChassis)
    {
        m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_world);
        m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);

        ///never deactivate the vehicle
        m_carChassis->setActivationState(DISABLE_DEACTIVATION);
        m_world->addVehicle(m_vehicle);

        //choose coordinate system
        m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

        bool isFrontWheel=true;
        btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
//        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        btVector3 A = connectionPointCS0;
        btVector3 B = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
        btVector3 C = (B+A)/2;
        m_vehicle->addWheel(C,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        isFrontWheel = false;
        A = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
//        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        B = connectionPointCS0;
        C = (B+A)/2;
        m_vehicle->addWheel(C,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        stepSimulation();
        return 0;//No error
    }
    return -1;//Error m_carChassis does not exist yet
}

void gameMoto::updateChassisTrans()
{
    #ifdef USE_OPTIMIZED_CODE
        //OPTIMMIZED CODE BELOW (LOOK AT THE UNOPTIMIZED CODE TO UNDERSTAND THIS SEGMENT CODE)
        //, matGround;
        ((btDefaultMotionState*)m_carChassis->getMotionState())->m_graphicsWorldTrans.getOpenGLMatrix(matChassis.pointer());

        static_cast<const btCompoundShape*>(m_carChassis->getCollisionShape())->getChildTransform(0).getOpenGLMatrix(matChassisChild.pointer());
        matChassis*=matChassisChild;
        chassis->setPosition(matChassis.getTranslation());
        chassis->setRotation(matChassis.getRotationDegrees());
    #else
    //USE SLOWER BUT CLEAR TO UNDERSTAND CODE FRAGMENT
//    btScalar	m[16];
        //    btMatrix3x3	rot;
        //    rot.setIdentity();

//            irr::core::matrix4 matChassisChild, matChassis;//, matGround;
            irr::f32* ptrChild,*ptrParent;//,*ptrGround;
            ptrChild = matChassisChild.pointer();
            ptrParent = matChassis.pointer();

        //        if(m_carChassis->getMotionState())
        //        {
        //            btDefaultMotionState* myMotionState = (btDefaultMotionState*)m_carChassis->getMotionState();
        //            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
        //            rot=myMotionState->m_graphicsWorldTrans.getBasis();
        //        }
            if(!m_carChassis->getMotionState())return;
            ///m_shapeDrawer->drawOpenGL(m,compound,wireColor,0,aabbMin,aabbMax);
            btDefaultMotionState* myMotionState = (btDefaultMotionState*)m_carChassis->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(ptrParent);

            btTransform childTrans = static_cast<const btCompoundShape*>(m_carChassis->getCollisionShape())->getChildTransform(0);
            const btCollisionShape* colShape = static_cast<const btCompoundShape*>(m_carChassis->getCollisionShape())->getChildShape(0);

            childTrans.getOpenGLMatrix(ptrChild);
            matChassis*=matChassisChild;
            chassis->setPosition(matChassis.getTranslation());
            chassis->setRotation(matChassis.getRotationDegrees());
            ///drawOpenGL(childMat,colShape,color,debugMode,worldBoundsMin,worldBoundsMax);
            ///m_shapeDrawer->drawCoordSystem();
    #endif // USE_OPTIMIZED_CODE
}

void gameMoto::moveVehicle()///moveAndDisplay()
{
    if(m_vehicle){
        int wheelIndex = 1;
        m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
        m_vehicle->setBrake(gBreakingForce,wheelIndex);

//        wheelIndex = 3;
//        m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
//        m_vehicle->setBrake(gBreakingForce,wheelIndex);

        wheelIndex = 0;
        m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

//        wheelIndex = 1;
//        m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

        updateVehicleTrans();
    }
    //float dt = getDeltaTimeMicroseconds() * 0.000001f;
}

void gameMoto::setWheelShape(const core::stringw& wheelMeshFile,
                                    const core::stringw& wheelTextureFile)
{
    m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
    IMesh* wheelMesh = m_smgr->getMesh(wheelMeshFile);//m_smgr->getMesh("models/roue.obj");
    for(int u=0;u<2;u++)
    {
        wl.push_back(m_smgr->addMeshSceneNode(wheelMesh));//m_smgr->addSphereSceneNode(wheelRadius,32);
        //wl[u]->setPosition(vector3df(0,wheelRadius,0));
        wl[wl.size()-1]->setMaterialFlag(EMF_LIGHTING,useLight);
        wl[wl.size()-1]->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture(wheelTextureFile));
        //wl[u]->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture("media/wheel.jpg"));
    }
}

void gameMoto::setChassisShape(const vector3df& chassisPosition,
                                          const vector3df& chassisRotation,
                                          const core::stringw& chassisTextureFile)
{
    //chassisWorldTrans.setIdentity();
    chassisWorldTrans.setOrigin(irr2bp::getBtVector(chassisPosition));
    //btQuaternion(yaw,pitch,roll);
    chassisWorldTrans.setRotation(btQuaternion(DEGTORAD64*chassisRotation.Y,
                                               DEGTORAD64*chassisRotation.X,
                                               DEGTORAD64*chassisRotation.Z));
    chassisShape = new btBoxShape(btVector3(0.25f,0.5f,2.f));
    //m_collisionShapes.push_back(chassisShape);
    compound = new btCompoundShape();
    //m_collisionShapes.push_back(compound);
    chassisLocalTrans.setIdentity();
    //localTrans effectively shifts the center of mass with respect to the chassis
    chassisLocalTrans.setOrigin(btVector3(0,1,0));
    compound->addChildShape(chassisLocalTrans,chassisShape);

    //chassisWorldTrans.setOrigin(btVector3(0,0,0));
    m_carChassis = localCreateRigidBody(200,chassisWorldTrans,compound);//chassisShape);
    //bodies.push_back(m_carChassis);
    //m_carChassis->setDamping(0.2,0.2);
    chassis = m_smgr->addCubeSceneNode(1.f);
    chassis->setScale(vector3df(0.25f,1,4.f));// = 2*(1.f,0.5f,2.f)
    chassis->setPosition(vector3df(0,0,0));
    chassis->setMaterialFlag(EMF_LIGHTING,useLight);
    chassis->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture(chassisTextureFile));
    //chassis->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture("media/fl.jpg"));

    scene::ILightSceneNode* light = m_smgr->addLightSceneNode(chassis,vector3df(0,20,0));
//    light->setParent(chassis);
}

bool gameMoto::init(const vector3df& position,const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile)
{
    #ifdef DEBUG_CONFIG
    cr = new chrono();
    #endif // DEBUG_CONFIG
    //setchassis
    setChassisShape(position,rotation,chassisTextureFile);
    //setWheekShape
    setWheelShape(wheelMeshFile,wheelTextureFile);
    //m_wheelShape = new btSphereShape(wheelRadius);
    resetVehicle();
    //create vehicle
    mountVehiclePieces();

    m_isReady = true;
    return true;
}
