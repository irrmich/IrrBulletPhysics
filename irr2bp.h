#ifndef IRR2BP_H_INCLUDED
#define IRR2BP_H_INCLUDED
#include "irrSimulation.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
namespace irr
{
    namespace physics
    {
        class irr2bp
        {
        public:
            /** \brief Create Bullet TriangleMeshShape from Irrlicht Mesh and scaling vector.
             *
             * \param	mesh Irrlicht mesh pointer.
             * \param	scale Scaling vector.
             * \return	Return a pointer to a new btTriangleMesh.
             *
             * Use this function to create static or kinematic bodies.
             */
            static btTriangleMesh* createTriangleMesh(const IMesh* mesh,const vector3df& scale);
            //static btTriangleMesh* createTriangleMeshEx(IMeshSceneNode* node);
            /** \brief Create Bullet ConvexHullShape from Irrlicht Mesh and scaling vector.
             *
             * \param	mesh Irrlicht mesh pointer.
             * \param	scale Scaling vector.
             * \return	Return a pointer to a new btConvexHullShape.
             *
             * For dynamic bodies.
             */
            static btConvexHullShape* createConvexHullShape(const IMesh* mesh,const vector3df& scale);
            //static btConvexHullShape* createConvexHullShapeEx(ISceneNode* node,IMesh* mesh);
            /** \brief Create an optimized Bullet ConvexHullShape from Irrlicht Mesh and scaling vector.
             *
             * \param	mesh Irrlicht mesh pointer.
             * \param	scale Scaling vector.
             * \return	Return a pointer to a new btConvexHullShape.
             *
             * It's optimized by using the triangle middle point(TM) or barycenter of each triangle.
             */
            static btConvexHullShape* createConvexHullShapeTM(const IMesh* mesh,const vector3df& scale);
            //transfBoundingBox = node->getTransformedBoundingBox()
            /** \brief Create an optimized Bullet from Irrlicht Mesh and scaling vector.
             *
             * \param	mesh Irrlicht mesh pointer.
             * \param	scale Scaling vector.
             * \return	Return a pointer to a new btConvexHullShape.
             *
             * It's optimized by using the bounding box of the mesh.
             */
            static btConvexHullShape* createConvexHullShapeBB(const aabbox3df& transfBoundingBox,
                                                                  const IMesh* mesh,const vector3df& scale);
            //static btConvexHullShape* createConvexHullShapeMBBB(ISceneNode* node,IMesh* mesh);
            /** \brief Create a Bullet Sphere shape from Irrlicht Mesh and scaling vector.
             *
             * \param	mesh Irrlicht mesh pointer.
             * \param	scale Scaling vector.
             * \return	Return a pointer to a new btConvexHullShape.
             *
             * It's optimized by using the bounding sphere of the mesh. Nice for kinematic characters
             * NB: TO BE IMPLEMENTED
             */
            static btConvexHullShape* createSphereShapeBS(const aabbox3df& transfBoundingBox,
                                                                  const IMesh* mesh,const vector3df& scale);
            static btVector3 getBtVector(const vector3df& v);
            static vector3df getIrrVector(const btVector3& v);
            static btTransform getBtTransform(const vector3df& position,const vector3df& rotation);
            static void getIrrTransform(vector3df& position,vector3df& rotation,const btTransform& trans);
        };
    }
}

#endif // IRR2BP_H_INCLUDED
