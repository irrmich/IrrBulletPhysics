#ifndef GAMEPLANE_H_INCLUDED
#define GAMEPLANE_H_INCLUDED
#include "gameSimpleCar.h"

namespace gameEngine
{
    class gamePlane:public gameSimpleCar
    {
    public:
        gamePlane();
        ~gamePlane();
    };
}


#endif // GAMEPLANE_H_INCLUDED
