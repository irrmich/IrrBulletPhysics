#ifndef IRRDEBUGDRAW_H_INCLUDED
#define IRRDEBUGDRAW_H_INCLUDED
#include "irrSimulation.h"


void coutVec(const irr::core::vector3df& v);
void coutVec(const irr::core::vector3df& v,const char* word);
void coutVec(const irr::core::vector3df& v,const std::string word);
void coutQuat(const irr::core::quaternion& q,const std::string word);
void addSkyBoxandDom(irr::video::IVideoDriver* driver, irr::scene::ISceneManager* smgr);

using namespace irr;
using namespace core;
using namespace video;
using namespace scene;

class irrDebugDraw
{
public:
    enum    DebugDrawModes
            {
                DBG_NoDebug=0,
                DBG_DrawWireframe = 1,
                DBG_DrawAabb=2,
                DBG_DrawFeaturesText=4,
                DBG_DrawContactPoints=8,
                DBG_NoDeactivation=16,
                DBG_NoHelpText = 32,
                DBG_DrawText=64,
                DBG_ProfileTimings = 128,
                DBG_EnableSatComparison = 256,
                DBG_DisableBulletLCP = 512,
                DBG_EnableCCD = 1024,
                DBG_DrawConstraints = (1 << 11),
                DBG_DrawConstraintLimits = (1 << 12),
                DBG_FastWireframe = (1<<13),
                DBG_DrawNormals = (1<<14),
                DBG_MAX_DEBUG_DRAW_MODE
            };

    irrDebugDraw(video::IVideoDriver* drv,scene::ISceneManager* smgr):m_driver(drv),m_smgr(smgr)
    {

    }

    ~irrDebugDraw()
    {

    }

    void drawAxes(btVector3 xColor,btVector3 yColor,btVector3 zColor,btScalar distance);

    btVector3 getBtVector(const vector3df& v)
    {
        return btVector3(v.X,v.Y,v.Z);
    }

    vector3df getIrrVector(const btVector3& v)
    {
        return vector3df(v.getX(),v.getY(),v.getZ());
    }

    void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color);
    void 	drawLine (const btVector3 &from, const btVector3 &to, const btVector3 &fromColor, const btVector3 &toColor)
    {

    }

    void 	drawSphere (btScalar radius, const btTransform &transform, const btVector3 &color)
    {

    }
    void 	drawSphere (const btVector3 &p, btScalar radius, const btVector3 &color)
    {

    }
    void 	drawTriangle (const btVector3 &v0, const btVector3 &v1, const btVector3 &v2,
                         const btVector3 &, const btVector3 &, const btVector3 &,
                          const btVector3 &color, btScalar alpha);
    void 	drawTriangle (const btVector3 &v0, const btVector3 &v1,
                         const btVector3 &v2, const btVector3 &color, btScalar)
    {

    }
    void 	drawContactPoint (const btVector3 &PointOnB,
                             const btVector3 &normalOnB, btScalar distance,
                              int lifeTime, const btVector3 &color)
    {

    }
    void 	reportErrorWarning (const char *warningString)
    {

    }
    void 	draw3dText (const btVector3 &location, const char *textString)
    {

    }
    void 	setDebugMode (int debugMode)
    {

    }
    int 	getDebugMode () const
    {

    }
    void 	drawAabb (const btVector3 &from, const btVector3 &to, const btVector3 &color)
    {

    }
    void 	drawTransform (const btTransform &transform, btScalar orthoLen)
    {

    }
    void 	drawArc (const btVector3 &center, const btVector3 &normal,
                    const btVector3 &axis, btScalar radiusA,
                     btScalar radiusB, btScalar minAngle, btScalar maxAngle,
                      const btVector3 &color, bool drawSect,
                       btScalar stepDegrees=btScalar(10.f))
    {

    }
    void 	drawSpherePatch (const btVector3 &center, const btVector3 &up,
                            const btVector3 &axis, btScalar radius,
                             btScalar minTh, btScalar maxTh, btScalar minPs,
                              btScalar maxPs, const btVector3 &color,
                               btScalar stepDegrees=btScalar(10.f), bool drawCenter=true)
    {

    }
    void 	drawBox (const btVector3 &bbMin, const btVector3 &bbMax,
                    const btVector3 &color);
    void 	drawBox (const btVector3 &bbMin, const btVector3 &bbMax,
                    const btTransform &trans, const btVector3 &color)
    {

    }

    void 	drawCapsule (btScalar radius, btScalar halfHeight,
                        int upAxis, const btTransform &transform, const btVector3 &color)
    {

    }
    void 	drawCylinder (btScalar radius, btScalar halfHeight,
                         int upAxis, const btTransform &transform, const btVector3 &color)
    {

    }
    void 	drawCone (btScalar radius, btScalar height, int upAxis,
                     const btTransform &transform, const btVector3 &color)
    {

    }
    void 	drawPlane (const btVector3 &planeNormal, btScalar planeConst,
                      const btTransform &transform, const btVector3 &color)
    {

    }
private:
    video::IVideoDriver* m_driver;
    scene::ISceneManager* m_smgr;
};


#endif // IRRDEBUGDRAW_H_INCLUDED
