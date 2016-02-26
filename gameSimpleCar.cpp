#include "gameSimpleCar.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace gameEngine;
/** \brief Default gameSimpleCar constructor.
 *
 * \param	world The Bullet world.
 * \param	position The position of the vehicle.
 * \param   rotation Rotation of the vehicle (NOT USED YET, NO EFFECT AT ALL)
 * \return	Return gameSimpleCar object pointer.
 *
 * Use this function to create a simple car, overload this class to create a better vehicle.
 */

gameSimpleCar::gameSimpleCar(btDynamicsWorld* world,ISceneManager* smgr,
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

void gameSimpleCar::setWheelShape(const core::stringw& wheelMeshFile,
                                    const core::stringw& wheelTextureFile)
{
    VehicleRaycastMeshNode::setWheelShape(wheelMeshFile,wheelTextureFile);
//    m_smgr->getMeshManipulator()->makePlanarTextureMapping(static_cast<IMeshSceneNode*>(wl[0])->getMesh(),0.08f);
}

void gameSimpleCar::setChassisShape(const vector3df& chassisPosition,
                                          const vector3df& chassisRotation,
                                          const core::stringw& chassisTextureFile)
{
    VehicleRaycastMeshNode::setChassisShape(chassisPosition,chassisRotation,chassisTextureFile);
    scene::ILightSceneNode* light = m_smgr->addLightSceneNode(chassis,vector3df(0,20,0));
//    light->setParent(chassis);
}

bool gameSimpleCar::init(const vector3df& position,const vector3df& rotation,
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

gameSimpleCar::~gameSimpleCar()
{

}
