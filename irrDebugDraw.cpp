#include "irrDebugDraw.h"
using namespace irr;
using namespace core;
using namespace video;
using namespace scene;
using namespace std;

void irrDebugDraw::drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color)
{
    video::SMaterial material;
    material.MaterialType = EMT_SOLID;
    material.EmissiveColor = SColor(color.getX(),color.getY(),color.getZ(),0);
    material.Wireframe = false;
    material.Thickness = 2.f;
    m_driver->setMaterial(material);
    core::matrix4 m = m_driver->getTransform(ETS_WORLD);
    m_driver->setTransform(ETS_WORLD,core::IdentityMatrix);
    m_driver->draw3DLine(getIrrVector(from),getIrrVector(to),
                         SColor(255,color.getX(),color.getY(),color.getZ()));
    m_driver->setTransform(ETS_WORLD,m);
}

void irrDebugDraw::drawAxes(btVector3 xColor,btVector3 yColor,btVector3 zColor,btScalar distance)
{
    drawLine(btVector3(0,0,0),distance*btVector3(1,0,0),xColor);
    drawLine(btVector3(0,0,0),distance*btVector3(0,1,0),yColor);
    drawLine(btVector3(0,0,0),distance*btVector3(0,0,1),zColor);
}

void coutVec(const vector3df& v)
{
    cout << "X=" << v.X << ", Y=" << v.Y << ", Z=" << v.Z << endl;
}

void coutVec(const irr::core::vector3df& v,const char* word)
{
    cout << word << endl;
    coutVec(v);
}

void coutVec(const irr::core::vector3df& v,const std::string word)
{
    cout << word << endl;
    coutVec(v);
}

void coutQuat(const irr::core::quaternion& q,const std::string word)
{
    cout << word << endl;
    cout << "q(X, Y, Z, W) = " << q.X << ", " << q.Y << ", " << q.Z << ", " << q.W << endl;
}

void addSkyBoxandDom(IVideoDriver* driver, ISceneManager* smgr)
{
    //driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);

    smgr->addSkyBoxSceneNode(
        driver->getTexture("media/irrlicht2_up.jpg"),
        driver->getTexture("media/irrlicht2_dn.jpg"),
        driver->getTexture("media/irrlicht2_lf.jpg"),
        driver->getTexture("media/irrlicht2_rt.jpg"),
        driver->getTexture("media/irrlicht2_ft.jpg"),
        driver->getTexture("media/irrlicht2_bk.jpg"));

    //driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);
}
