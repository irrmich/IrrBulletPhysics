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
      init(position,rotation,wheelMeshFile,wheelTextureFile,chassisTextureFile);
}

gameSimpleCar::~gameSimpleCar()
{

}
