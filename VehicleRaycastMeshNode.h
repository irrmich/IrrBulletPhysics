#ifndef VEHICLERAYCASTMESHNODE_H_INCLUDED
#define VEHICLERAYCASTMESHNODE_H_INCLUDED
#include "irrSimulation.h"
#include "irr2bp.h"
//#include "irrPhysics.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

class VehicleRaycastMeshNode
{
public:
    #define CUBE_HALF_EXTENTS 1
    VehicleRaycastMeshNode(btDynamicsWorld* world,ISceneManager* smgr,
                              const vector3df& position=vector3df(0,0,0),
                              const vector3df& rotation=vector3df(0,0,0),
                              const core::stringw& wheelMeshFile="models/roue.obj",
                              const core::stringw& wheelTextureFile="media/wheel.jpg",
                              const core::stringw& chassisTextureFile="media/fl.jpg");
    vector3df getPosition();
    int resetVehicle();
    int restoreVehicleSteering();
    void stepSimulation();
    void moveVehicle();
    bool setCameraFollower(ICameraSceneNode* cam);
    bool isReady() const;
    bool OnEvent(const SEvent &event);
    ~VehicleRaycastMeshNode();

protected:
    bool init(const vector3df& position,
              const vector3df& rotation,
              const core::stringw& wheelMeshFile,
              const core::stringw& wheelTextureFile,
              const core::stringw& chassisTextureFile);
    void setChassisShape(const vector3df& chassisPosition=vector3df(0,0,0),
                         const vector3df& chassisRotation=vector3df(0,0,0),
                        const core::stringw& chassisTextureFile="");
    void setWheelShape(const core::stringw& wheelMeshFile,
                        const core::stringw& wheelTextureFile);
    btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);
    int mountVehiclePieces();
    void updateChassisTrans();
    void updateVehicleTrans();
    void updateSteeringClamp();
//    void drop();
protected:
    #ifdef DEBUG_CONFIG
        chrono* cr;
    #endif // DEBUG_CONFIG
    irr::core::matrix4 matChassisChild, matChassis;

    std::vector<ISceneNode*> wl;
    ISceneNode* chassis = 0;

    bool m_isReady;
    bool useLight = false;

    btCollisionShape* chassisShape = 0;
    btRigidBody* m_carChassis = 0;
    btCompoundShape* compound = 0;
    //btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btRaycastVehicle::btVehicleTuning	m_tuning;
    btVehicleRaycaster*	m_vehicleRayCaster = 0;
    btRaycastVehicle*	m_vehicle = 0;
    btCollisionShape*	m_wheelShape = 0;

    btTransform chassisWorldTrans;
    btTransform chassisLocalTrans;

    ///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
    int rightIndex = 0;
    int upIndex = 1;
    int forwardIndex = 2;

    btVector3 wheelDirectionCS0 = btVector3(0,-1,0);
    btVector3 wheelAxleCS = btVector3(-1,0,0);

    float	gEngineForce = 0.f;
    float	gBreakingForce = 0.f;

    float	maxEngineForce = 1000.f;//1000.f;//this should be engine/velocity dependent
    float	maxBreakingForce = 100.f;

    float   gBreakingIncrement = 20.f; //(ajouté par michael) Incrémenter le niveau de freinage
    float	gVehicleSteering = 0.f;
    float	steeringIncrement = 0.04f;//(michael) par defaut 0.04f; Ceci améliore ou détériore la réactivité du volon de l'auto
    float	steeringClamp = 0.3f;
    float	wheelRadius = 0.5f;	//(michael) rayon d'une roue
    float	wheelWidth = 0.2f; //(michael) épaisseur d'une roue
    float	wheelFriction = 1000;//BT_LARGE_FLOAT;
    float	suspensionStiffness = 20.f;//(michael) raideur, souplesse ou flexibilité,
                                        // plus le raideur est élevé plus l'état de l'amortisseur est dur à changer
    float	suspensionDamping = 10.3f;//Coefficient d'amortissement (plus c'est élevé, plus l'amortisseur met du temps à osciller)
    float	suspensionCompression = 4.4f;
    float	rollInfluence = 0.4f;/*1.0f;(michael) Quand cette valeur est élevée, un clic sur tourner à droite ou à gauche fait
                                figer le mobile dans cette direction pendant un temps de plus en plus longtemps
                                On remarque aussi que plus la valeur est élevée, le coin d'un des roues devant
                                tend à s'incliner devant l'autre*/
    float connectionHeight = 1.2f;//(michael) augmente ou diminue la longueur de l'amortisseur
    float suspensionRestLength = 0.7f; //(Michael) Apparement, ceci correspond à la longueur de l'amortisseur (defaut: 0.6)
    bool bRestoreSteering = false;

    btDynamicsWorld* m_world = 0;
    ISceneManager* m_smgr = 0;
};


#endif // VEHICLERAYCASTMESHNODE_H_INCLUDED
