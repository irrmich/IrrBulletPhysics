#include "irr2bp.h"
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace physics;

btTriangleMesh* irr2bp::createTriangleMesh(const IMesh* mesh,const vector3df& scale)
{
    btVector3 vertices[3];
	u32 i, j, k;
	s32 index, numVertices;
	u16* mb_indices;

	btTriangleMesh *pTriMesh = new btTriangleMesh();

	for(i = 0; i < mesh->getMeshBufferCount(); i++)
	{
		irr::scene::IMeshBuffer* mb=mesh->getMeshBuffer(i);

        //////////////////////////////////////////////////////////////////////////
		// Extract vertex data                                                  //
		// Because the vertices are stored as structs with no common base class,//
		// We need to handle each type separately                               //
		//////////////////////////////////////////////////////////////////////////
		if(mb->getVertexType() == irr::video::EVT_STANDARD)
		{
			irr::video::S3DVertex* mb_vertices=(irr::video::S3DVertex*)mb->getVertices();
			mb_indices = mb->getIndices();
			numVertices = mb->getVertexCount();
			for(j=0;j<mb->getIndexCount();j+=3)
			{ //get index into vertex list
				for (k=0;k<3;k++)
				{
				    //three verts per triangle
					index = mb_indices[j+k];
					if (index > numVertices) continue;
					//convert to btVector3
					vertices[k] = getBtVector(mb_vertices[index].Pos * scale); // 1100
				}
				pTriMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
			}

		}
		else
		if(mb->getVertexType()==irr::video::EVT_2TCOORDS)
		{
			// Same but for S3DVertex2TCoords data
			irr::video::S3DVertex2TCoords* mb_vertices=(irr::video::S3DVertex2TCoords*)mb->getVertices();
			u16* mb_indices = mb->getIndices();
			s32 numVertices = mb->getVertexCount();
			for(j=0;j<mb->getIndexCount();j+=3)
			{ //index into irrlicht data
				for (k=0;k<3;k++)
				{
					s32 index = mb_indices[j+k];
					if (index > numVertices) continue;
					vertices[k] = getBtVector(mb_vertices[index].Pos * scale);
				}
				pTriMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
			}
		}

		// Does not handle the EVT_TANGENTS type
	}

	return pTriMesh;
}

/*btTriangleMesh* irr2bp::createTriangleMeshEx(IMeshSceneNode* node)
{
    ITriangleSelector* sel = smgr->createOctreeTriangleSelector(node->getMesh(),node);
    s32 triangleCount(sel->getTriangleCount());
    triangle3df triangles[triangleCount];
    matrix4 m;

    m.setScale(node->getScale());
    sel->getTriangles(triangles,triangleCount,triangleCount,&m);
    sel->drop();

    btTriangleMesh* pTriMesh = new btTriangleMesh();
    for(unsigned i=0;i<triangleCount;i++)
    {
        pTriMesh->addTriangle(getBtVector(triangles[i].pointA),
                              getBtVector(triangles[i].pointB),
                              getBtVector(triangles[i].pointC));
    }

    return pTriMesh;
}*/

btConvexHullShape* irr2bp::createConvexHullShape(const IMesh* mesh,const vector3df& scale)
{
btConvexHullShape* convShape = new btConvexHullShape();

    btVector3 vertices[3];
	u32 i, j, k;
	s32 index, numVertices;
	u16* mb_indices;

	for(i = 0; i < mesh->getMeshBufferCount(); i++)
	{
		irr::scene::IMeshBuffer* mb=mesh->getMeshBuffer(i);

        //////////////////////////////////////////////////////////////////////////
		// Extract vertex data                                                  //
		// Because the vertices are stored as structs with no common base class,//
		// We need to handle each type separately                               //
		//////////////////////////////////////////////////////////////////////////
		if(mb->getVertexType() == irr::video::EVT_STANDARD)
		{
			irr::video::S3DVertex* mb_vertices=(irr::video::S3DVertex*)mb->getVertices();
			mb_indices = mb->getIndices();
			numVertices = mb->getVertexCount();
			for(j=0;j<mb->getIndexCount();j+=3)
			{ //get index into vertex list
				for (k=0;k<3;k++)
				{
				    //three verts per triangle
					index = mb_indices[j+k];
					if (index > numVertices) continue;
					//convert to btVector3
					vertices[k] = getBtVector(mb_vertices[index].Pos * scale); // 1100
				}

				convShape->addPoint(vertices[0]);
				convShape->addPoint(vertices[1]);
				convShape->addPoint(vertices[2]);
			}

		}
		else
		if(mb->getVertexType()==irr::video::EVT_2TCOORDS)
		{
			// Same but for S3DVertex2TCoords data
			irr::video::S3DVertex2TCoords* mb_vertices=(irr::video::S3DVertex2TCoords*)mb->getVertices();
			u16* mb_indices = mb->getIndices();
			s32 numVertices = mb->getVertexCount();
			for(j=0;j<mb->getIndexCount();j+=3)
			{ //index into irrlicht data
				for (k=0;k<3;k++)
				{
					s32 index = mb_indices[j+k];
					if (index > numVertices) continue;
					vertices[k] = getBtVector(mb_vertices[index].Pos * scale);
				}

				convShape->addPoint(vertices[0]);
				convShape->addPoint(vertices[1]);
				convShape->addPoint(vertices[2]);
			}
		}

		// Does not handle the EVT_TANGENTS type
	}

	return convShape;
}

/*btConvexHullShape* irr2bp::createConvexHullShapeEx(ISceneNode* node,IMesh* mesh)
{
ITriangleSelector* sel = smgr->createOctreeTriangleSelector(node->getMesh(),node);
    s32 triangleCount(sel->getTriangleCount());
    triangle3df triangles[triangleCount];
    matrix4 m;

    m.setScale(node->getScale());
    sel->getTriangles(triangles,triangleCount,triangleCount,&m);
    sel->drop();

    btTriangleMesh* pTriMesh = new btTriangleMesh();
    for(unsigned i=0;i<triangleCount;i++)
    {
        pTriMesh->addTriangle(getBtVector(triangles[i].pointA),
                              getBtVector(triangles[i].pointB),
                              getBtVector(triangles[i].pointC));
    }

    return pTriMesh;
}*/

btConvexHullShape* irr2bp::createConvexHullShapeTM(const IMesh* mesh,const vector3df& scale)
{
    btConvexHullShape* convShape = new btConvexHullShape();

    btVector3 vertices[3];
	u32 i, j, k;
	s32 index, numVertices;
	u16* mb_indices;

	for(i = 0; i < mesh->getMeshBufferCount(); i++)
	{
		irr::scene::IMeshBuffer* mb=mesh->getMeshBuffer(i);

        //////////////////////////////////////////////////////////////////////////
		// Extract vertex data                                                  //
		// Because the vertices are stored as structs with no common base class,//
		// We need to handle each type separately                               //
		//////////////////////////////////////////////////////////////////////////
		if(mb->getVertexType() == irr::video::EVT_STANDARD)
		{
			irr::video::S3DVertex* mb_vertices=(irr::video::S3DVertex*)mb->getVertices();
			mb_indices = mb->getIndices();
			numVertices = mb->getVertexCount();
			for(j=0;j<mb->getIndexCount();j+=3)
			{ //get index into vertex list
				for (k=0;k<3;k++)
				{
				    //three verts per triangle
					index = mb_indices[j+k];
					if (index > numVertices) continue;
					//convert to btVector3
					vertices[k] = getBtVector(mb_vertices[index].Pos * scale); // 1100
				}

				convShape->addPoint((vertices[0]+vertices[1]+vertices[2])/3);
			}
		}
		else
		if(mb->getVertexType()==irr::video::EVT_2TCOORDS)
		{
			// Same but for S3DVertex2TCoords data
			irr::video::S3DVertex2TCoords* mb_vertices=(irr::video::S3DVertex2TCoords*)mb->getVertices();
			u16* mb_indices = mb->getIndices();
			s32 numVertices = mb->getVertexCount();
			for(j=0;j<mb->getIndexCount();j+=3)
			{ //index into irrlicht data
				for (k=0;k<3;k++)
				{
					s32 index = mb_indices[j+k];
					if (index > numVertices) continue;
					vertices[k] = getBtVector(mb_vertices[index].Pos * scale);
				}

				convShape->addPoint((vertices[0]+vertices[1]+vertices[2])/3);
			}
		}

		// Does not handle the EVT_TANGENTS type
	}

	return convShape;
}

btConvexHullShape* irr2bp::createConvexHullShapeBB(const aabbox3df& transfBoundingBox,
                                                              const IMesh* mesh,const vector3df& scale)
{
    btConvexHullShape* convShape = new btConvexHullShape();
    vector3df edges[8];
    transfBoundingBox.getEdges(edges);
    for(int i=0;i<8;i++)
    {
        convShape->addPoint(getBtVector(edges[i]*scale));
    }
    return convShape;
}

/*btConvexHullShape* irr2bp::createConvexHullShapeMBBB(ISceneNode* node,IMesh* mesh)
{

}*/

btVector3 irr2bp::getBtVector(const vector3df& v)
{
    return btVector3(v.X,v.Y,v.Z);
}

vector3df irr2bp::getIrrVector(const btVector3& v)
{
    return vector3df(v.getX(),v.getY(),v.getZ());
}

btTransform irr2bp::getBtTransform(const vector3df& position,const vector3df& rotation)
{
    core::quaternion quatX,quatY,quatZ,quatF;
    quatX.fromAngleAxis(rotation.X*DEGTORAD64,vector3df(1,0,0));
    quatY.fromAngleAxis(rotation.Y*DEGTORAD64,vector3df(0,1,0));
    quatZ.fromAngleAxis(rotation.Z*DEGTORAD64,vector3df(0,0,1));

    quatF = quatX*quatY*quatZ;

    btQuaternion btquat;
    btquat.setX(quatF.X);
    btquat.setY(quatF.Y);
    btquat.setZ(quatF.Z);
    btquat.setW(quatF.W);

    btTransform* trans = new btTransform(btquat,getBtVector(position));
    return *trans;
}

void irr2bp::getIrrTransform(vector3df& position,vector3df& rotation,const btTransform& trans)
{
    btQuaternion orientation = trans.getRotation();
    btVector3 pos = trans.getOrigin();
    quaternion quat(orientation.getX(),orientation.getY(),orientation.getZ(),orientation.getW());

    quat.toEuler(rotation);
    position = getIrrVector(pos);
    rotation*=RADTODEG64;
}
