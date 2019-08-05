#ifndef APHY_COLLISION_MESH_COLLECTION_HPP
#define APHY_COLLISION_MESH_COLLECTION_HPP

#include "Common.hpp"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"

namespace APhyBullet
{

class APhyBulletCollisionMeshCollection : public collision_mesh_collection
{
public:
    APhyBulletCollisionMeshCollection();
    ~APhyBulletCollisionMeshCollection();

	virtual aphy_error addCollisionMeshAccessor(aphy_collision_mesh_accessor* accessor, aphy_transform* transform) override;

    btTriangleMesh triangleMeshData;

private:
    aphy_error addCollisionMeshAccessorWithTransform(aphy_collision_mesh_accessor* accessor, const btTransform &transform);
};

} // End of namespace APhyBullet

#endif //APHY_COLLISION_MESH_COLLECTION_HPP
