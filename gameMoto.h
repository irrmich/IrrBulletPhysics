#ifndef GAMEMOTO_H_INCLUDED
#define GAMEMOTO_H_INCLUDED
#include "gameSimpleCar.h"

namespace gameEngine
{
    class gameMoto:public VehicleRaycastMeshNode
    {
    public:
        gameMoto(btDynamicsWorld* world,ISceneManager* smgr,
                              const vector3df& position,
                              const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile);
        bool init(const vector3df& position,const vector3df& rotation,
                              const core::stringw& wheelMeshFile,
                              const core::stringw& wheelTextureFile,
                              const core::stringw& chassisTextureFile);
        void setWheelShape(const core::stringw& wheelMeshFile,
                           const core::stringw& wheelTextureFile);
        void setChassisShape(const vector3df& chassisPosition,
                              const vector3df& chassisRotation,
                              const core::stringw& chassisTextureFile);
        void moveVehicle();
        void updateChassisTrans();
        virtual int mountVehiclePieces();
        ~gameMoto();
    };
}

#endif // GAMEMOTO_H_INCLUDED
