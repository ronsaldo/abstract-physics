#ifndef APHY_UTILITY_HPP
#define APHY_UTILITY_HPP

#include "common.hpp"
#include "btBulletDynamicsCommon.h"

inline aphy_vector3 convertVector(const btVector3 &vector)
{
    return {vector.x(), vector.y(), vector.z()};
}

inline aphy_quaternion convertQuaternion(const btQuaternion &quaternion)
{
    return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

inline aphy_matrix3x3 convertMatrix(const btMatrix3x3 &matrix)
{
    return {convertVector(matrix.getRow(0)), convertVector(matrix.getRow(1)), convertVector(matrix.getRow(2))};
}

inline aphy_transform convertTransform(const btTransform &transform)
{
    return {convertMatrix(transform.getBasis()), convertVector(transform.getOrigin())};
}
#endif //APHY_UTILITY_HPP
