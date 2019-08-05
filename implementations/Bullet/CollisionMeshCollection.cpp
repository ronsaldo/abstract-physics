#include "CollisionMeshCollection.hpp"
#include "Utility.hpp"

namespace APhyBullet
{

APhyBulletCollisionMeshCollection::APhyBulletCollisionMeshCollection()
{
}

APhyBulletCollisionMeshCollection::~APhyBulletCollisionMeshCollection()
{
}

aphy_error APhyBulletCollisionMeshCollection::addCollisionMeshAccessor(aphy_collision_mesh_accessor* accessor, aphy_transform* transform)
{
    CHECK_POINTER(accessor);

    if(transform)
        return addCollisionMeshAccessorWithTransform(accessor, convertAPhyTransform(*transform));

    btIndexedMesh indexedMesh;
    indexedMesh.m_numTriangles = accessor->index_count / 3;
    indexedMesh.m_triangleIndexBase = reinterpret_cast<uint8_t*> (accessor->indices) + accessor->index_offset;
    indexedMesh.m_triangleIndexStride = accessor->index_stride * 3;
    indexedMesh.m_numVertices = accessor->vertex_count;
    indexedMesh.m_vertexBase = reinterpret_cast<uint8_t*> (accessor->vertices) + accessor->vertex_offset;
    indexedMesh.m_vertexStride = accessor->vertex_stride;
    indexedMesh.m_vertexType = PHY_FLOAT;
    //printf("m_vertexBase %p m_triangleIndexBase %p\n", indexedMesh.m_vertexBase, indexedMesh.m_triangleIndexBase);
    //printf("m_numTriangles %d m_triangleIndexStride %d m_numVertices %d m_vertexStride %d\n", indexedMesh.m_numTriangles, indexedMesh.m_triangleIndexStride, indexedMesh.m_numVertices, indexedMesh.m_vertexStride);
    triangleMeshData.addIndexedMesh(indexedMesh, accessor->index_stride == 2 ? PHY_SHORT : PHY_INTEGER);
    return APHY_OK;
}

aphy_error APhyBulletCollisionMeshCollection::addCollisionMeshAccessorWithTransform(aphy_collision_mesh_accessor* accessor, const btTransform &transform)
{
    auto rawIndices = reinterpret_cast<const uint8_t*> (accessor->indices);
    auto rawVertices = reinterpret_cast<const uint8_t*> (accessor->vertices);
    size_t triangleCount = accessor->index_count / 3;
    if(accessor->index_stride == 2)
    {
        auto indices = reinterpret_cast<const uint16_t*> (rawIndices + accessor->index_offset);
        for(size_t t = 0; t < triangleCount; ++t, indices +=3)
        {
            auto i1 = indices[0];
            auto i2 = indices[1];
            auto i3 = indices[2];

            auto v1 = reinterpret_cast<const float*> (rawVertices + accessor->vertex_offset + i1*accessor->vertex_stride);
            auto v2 = reinterpret_cast<const float*> (rawVertices + accessor->vertex_offset + i2*accessor->vertex_stride);
            auto v3 = reinterpret_cast<const float*> (rawVertices + accessor->vertex_offset + i3*accessor->vertex_stride);
            triangleMeshData.addTriangle(
                transform*btVector3(v1[0], v1[1], v1[2]),
                transform*btVector3(v2[0], v2[1], v2[2]),
                transform*btVector3(v3[0], v3[1], v3[2]));
        }

    }
    else if(accessor->index_stride == 4)
    {
        auto indices = reinterpret_cast<const uint32_t*> (rawIndices + accessor->index_offset);
        for(size_t t = 0; t < triangleCount; ++t, indices +=3)
        {
            auto i1 = indices[0];
            auto i2 = indices[1];
            auto i3 = indices[2];

            auto v1 = reinterpret_cast<const float*> (rawVertices + accessor->vertex_offset + i1*accessor->vertex_stride);
            auto v2 = reinterpret_cast<const float*> (rawVertices + accessor->vertex_offset + i2*accessor->vertex_stride);
            auto v3 = reinterpret_cast<const float*> (rawVertices + accessor->vertex_offset + i3*accessor->vertex_stride);
            triangleMeshData.addTriangle(
                transform*btVector3(v1[0], v1[1], v1[2]),
                transform*btVector3(v2[0], v2[1], v2[2]),
                transform*btVector3(v3[0], v3[1], v3[2]));
        }
    }

    return APHY_OK;
}

} // End of namespace APhyBullet
