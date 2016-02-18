#ifndef GAMESIMPLECAR_H_INCLUDED
#define GAMESIMPLECAR_H_INCLUDED
#include "irrSimulation.h"
#include "VehicleRaycastMeshNode.h"
//#include "irrEventTracker.h"
#include "irrSong.h"
#include "irrNetwork.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

namespace gameEngine
{
    class gameSimpleCar:public VehicleRaycastMeshNode
    {
    public:
        gameSimpleCar(btDynamicsWorld* world,ISceneManager* smgr,
                              const vector3df& position,
                              const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile);
        ~gameSimpleCar();
    protected:
        int m_id;
    };
}


#endif // GAMESIMPLECAR_H_INCLUDED
