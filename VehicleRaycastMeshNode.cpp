//#define CUBE_HALF_EXTENTS 1
#include "VehicleRaycastMeshNode.h"
#define USE_OPTIMIZED_CODE
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace physics;

VehicleRaycastMeshNode::VehicleRaycastMeshNode(btDynamicsWorld* world,ISceneManager* smgr,
                              const vector3df& position,
                              const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile)
{
    m_world = world;
    m_smgr = smgr;
    m_isReady = init(position,rotation,wheelMeshFile,wheelTextureFile,chassisTextureFile);
}

bool VehicleRaycastMeshNode::isReady() const
{
    return m_isReady;
}

bool VehicleRaycastMeshNode::init(const vector3df& position,const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile)
{
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

vector3df VehicleRaycastMeshNode::getPosition()
{
    if(m_vehicle)
    {
        btVector3 pos = (m_vehicle->getChassisWorldTransform().getOrigin());
        return vector3df(pos.getX(),pos.getY(),pos.getZ());
    }
}

int VehicleRaycastMeshNode::resetVehicle()
{
    if (m_vehicle)
    {
        gVehicleSteering = 0.f;
        m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
        m_carChassis->setLinearVelocity(btVector3(0,0,0));
        m_carChassis->setAngularVelocity(btVector3(0,0,0));
        m_world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),
                                                                            m_world->getDispatcher());
        m_vehicle->resetSuspension();
        for (int i=0;i<m_vehicle->getNumWheels();i++)
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            m_vehicle->updateWheelTransform(i,true);
        }
        return 0;//No error
    }
    return -1;//Error m_carChassis does not exist yet
}

int VehicleRaycastMeshNode::restoreVehicleSteering()
{
    /*if(abs(gVehicleSteering)<(steeringIncrement))
    {
        return 0;
    }*/
    if(bRestoreSteering)
    {
        if(gVehicleSteering>0)
        {
            if((gVehicleSteering-(steeringIncrement/20.f))>0)
            {
                gVehicleSteering-=(steeringIncrement/20.f);
            } else
            {
                gVehicleSteering=0;
            }
        }

        if(gVehicleSteering<0)
        {
            if((gVehicleSteering+(steeringIncrement/20.f))<0)
            {
                gVehicleSteering+=(steeringIncrement/20.f);
            } else
            {
                gVehicleSteering=0;
            }
        }
    }
}

void VehicleRaycastMeshNode::stepSimulation()
{
    if(m_vehicle)
    {
        for (int i=0;i<m_vehicle->getNumWheels();i++)
        {
            btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = suspensionStiffness;
            wheel.m_wheelsDampingRelaxation = suspensionDamping;
            wheel.m_wheelsDampingCompression = suspensionCompression;
            wheel.m_frictionSlip = wheelFriction;
            wheel.m_rollInfluence = rollInfluence;
        }
    }
}

void VehicleRaycastMeshNode::moveVehicle()///moveAndDisplay()
{
    if(m_vehicle){
        int wheelIndex = 2;
        m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
        m_vehicle->setBrake(gBreakingForce,wheelIndex);

        wheelIndex = 3;
        m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
        m_vehicle->setBrake(gBreakingForce,wheelIndex);

        wheelIndex = 0;
        m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

        wheelIndex = 1;
        m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

        updateVehicleTrans();
    }
    //float dt = getDeltaTimeMicroseconds() * 0.000001f;
}

bool VehicleRaycastMeshNode::setCameraFollower(ICameraSceneNode* cam)
{
    vector3df m_cameraTargetPosition = chassis->getPosition();
    vector3df m_cameraPosition = cam->getPosition();
    float m_cameraHeight = 10.f;
    //interpolate the camera height

    m_cameraPosition.Y = (15.0*m_cameraPosition.Y + m_cameraTargetPosition.Y + m_cameraHeight)/16.0;

    vector3df camToObject = m_cameraTargetPosition - m_cameraPosition;

    //keep distance between min and max distance
    float cameraDistance = camToObject.getLength();
    float correctionFactor = 0.f;
    float m_minCameraDistance=5.f;
    float m_maxCameraDistance=6.f;
    if (cameraDistance < m_minCameraDistance)
    {
        correctionFactor = 0.15*(m_minCameraDistance-cameraDistance)/cameraDistance;
    }
    if (cameraDistance > m_maxCameraDistance)
    {
        correctionFactor = 0.15*(m_maxCameraDistance-cameraDistance)/cameraDistance;
    }
    m_cameraPosition -= correctionFactor*camToObject;

    cam->setPosition(m_cameraPosition);
    cam->setTarget(m_cameraTargetPosition);
}

bool VehicleRaycastMeshNode::OnEvent(const SEvent &event)
{
    if(event.EventType == EET_KEY_INPUT_EVENT){
        if(event.KeyInput.PressedDown) {
            switch(event.KeyInput.Key)
            {
                case KEY_NUMPAD4:
                    bRestoreSteering=false;
                    updateSteeringClamp();
                    gVehicleSteering -= steeringIncrement;
                    if (gVehicleSteering < -steeringClamp)
                        gVehicleSteering = -steeringClamp;
                    break;
                case KEY_NUMPAD6:
                    bRestoreSteering=false;
                    updateSteeringClamp();
                    gVehicleSteering += steeringIncrement;
                    if (gVehicleSteering > steeringClamp)
                        gVehicleSteering = steeringClamp;
                    break;
                case KEY_NUMPAD8:
                    gEngineForce = maxEngineForce;
                    gBreakingForce = 0.f;
                    break;
                case KEY_NUMPAD2:
                    if((m_vehicle)&&(m_vehicle->getCurrentSpeedKmHour()<=0))
                    {
                        gEngineForce = -maxEngineForce;
                        gBreakingForce = 0.f;
                    } else
                    {
                        gBreakingForce = maxBreakingForce;
                        gEngineForce = 0.f;
                    }
                    break;
                case KEY_NUMPAD5:
                    gEngineForce = 0.f;
                    if(maxBreakingForce>gBreakingForce)
                    {
                        gBreakingForce+=gBreakingIncrement;
                    }
                    break;
                default:
                    break;
            }
        } else {
            switch(event.KeyInput.Key)
            {
            //VEHICLE TEST
            case KEY_NUMPAD4:
                bRestoreSteering=true;
                break;
            case KEY_NUMPAD6:
                bRestoreSteering=true;
                break;
            case KEY_NUMPAD8:
                gEngineForce = 0.f;
                //gBreakingForce = maxBreakingForce/2;
                break;
            case KEY_NUMPAD2:
                if((m_vehicle)&&(m_vehicle->getCurrentSpeedKmHour()<=0))
                    {
                        gEngineForce = 0.f;
                        //gBreakingForce = maxBreakingForce;
                    }
                break;
            default:
                break;
            }
        }
    }
    return false;
}

VehicleRaycastMeshNode::~VehicleRaycastMeshNode()
{
    if (m_carChassis->getMotionState())
    {
        delete m_carChassis->getMotionState();
    }

    m_world->removeRigidBody(m_carChassis);

    delete chassisShape;
    delete compound;
    delete m_vehicleRayCaster;
    delete m_vehicle;
    delete m_wheelShape;

    for(int i=0;i<m_vehicle->getNumWheels();i++)
    {
        wl[i]->remove();
    }
    chassis->remove();
}
//protected part
void VehicleRaycastMeshNode::setChassisShape(const vector3df& chassisPosition,
                                          const vector3df& chassisRotation,
                                          const core::stringw& chassisTextureFile)
{
    //chassisWorldTrans.setIdentity();
    chassisWorldTrans.setOrigin(irr2bp::getBtVector(chassisPosition));
    //btQuaternion(yaw,pitch,roll);
    chassisWorldTrans.setRotation(btQuaternion(DEGTORAD64*chassisRotation.Y,
                                               DEGTORAD64*chassisRotation.X,
                                               DEGTORAD64*chassisRotation.Z));
    chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
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
    chassis->setScale(vector3df(2,1,4));// = 2*(1.f,0.5f,2.f)
    chassis->setPosition(vector3df(0,0,0));
    chassis->setMaterialFlag(EMF_LIGHTING,useLight);
    chassis->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture(chassisTextureFile));
    //chassis->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture("media/fl.jpg"));
}

void VehicleRaycastMeshNode::setWheelShape(const core::stringw& wheelMeshFile,
                                        const core::stringw& wheelTextureFile)
{
    m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
    IMesh* wheelMesh = m_smgr->getMesh(wheelMeshFile);//m_smgr->getMesh("models/roue.obj");
    for(int u=0;u<4;u++)
    {
        wl[u] = m_smgr->addMeshSceneNode(wheelMesh);//m_smgr->addSphereSceneNode(wheelRadius,32);
        //wl[u]->setPosition(vector3df(0,wheelRadius,0));
        wl[u]->setMaterialFlag(EMF_LIGHTING,useLight);
        wl[u]->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture(wheelTextureFile));
        //wl[u]->setMaterialTexture(0,m_smgr->getVideoDriver()->getTexture("media/wheel.jpg"));
    }
}

btRigidBody* VehicleRaycastMeshNode::localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
    //btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);

    m_world->addRigidBody(body);
    return body;
}

int VehicleRaycastMeshNode::mountVehiclePieces()
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
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        isFrontWheel = false;
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

        stepSimulation();
        return 0;//No error
    }
    return -1;//Error m_carChassis does not exist yet
}

void VehicleRaycastMeshNode::updateChassisTrans()
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
    //USE SLOW BUT CLEAR TO UNDERSTAND CODE FRAGMENT
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

void VehicleRaycastMeshNode::updateVehicleTrans()
{
    //btScalar m[16];
//    int i;

    //btVector3 wheelColor(1,0,0);
//    btVector3	worldBoundsMin,worldBoundsMax;
//    m_world->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);

    irr::core::matrix4 mat;
    irr::f32 * ptr;
    ptr = mat.pointer();

    for (int i=0;i<m_vehicle->getNumWheels();i++)
    {
        //synchronize the wheels with the (interpolated) chassis worldtransform
        m_vehicle->updateWheelTransform(i,true);
        //draw wheels (cylinders)
            ///m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
            ///m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,0,worldBoundsMin,worldBoundsMax);
        ///TO DO render the wheel shapes
        wl[i]->setPosition(vector3df(   m_vehicle->getWheelInfo(i).m_worldTransform.getOrigin().getX(),
                                        m_vehicle->getWheelInfo(i).m_worldTransform.getOrigin().getY(),
                                        m_vehicle->getWheelInfo(i).m_worldTransform.getOrigin().getZ()
                                    )
                           );
        m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(ptr);
        wl[i]->setRotation(mat.getRotationDegrees());
    }

    if (m_world)
    {
        //glDisable(GL_CULL_FACE);
        updateChassisTrans();
    }
}

void VehicleRaycastMeshNode::updateSteeringClamp()
{
    steeringClamp=30.f/(120+3*abs(m_vehicle->getCurrentSpeedKmHour()));
}

//void VehicleRaycastMeshNode::drop()
//{
//    for(int i=0;i<4;i++)
//    {
//        wl[i]->remove();
//    }
//    chassis->remove();
//    m_world->removeAction(m_vehicle);
//    delete m_vehicle;
//    delete m_vehicleRayCaster;
//    m_world->removeCollisionObject(m_carChassis);
//    m_world->removeCollisionObject(compound);
//    m_world->removeCollisionObject(chassisShape);
//    delete m_carChassis;
//    delete compound;
//    delete chassisShape;
//}
