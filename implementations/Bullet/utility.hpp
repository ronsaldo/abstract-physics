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
    return {quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
}

inline aphy_matrix3x3 convertMatrix(const btMatrix3x3 &matrix)
{
    return {convertVector(matrix.getRow(0)), convertVector(matrix.getRow(1)), convertVector(matrix.getRow(2))};
}

inline aphy_transform convertTransform(const btTransform &transform)
{
    return {convertMatrix(transform.getBasis()), convertVector(transform.getOrigin())};
}

inline btVector3 convertAPhyVector(const aphy_vector3 &vector)
{
    return btVector3(vector.x, vector.y, vector.z);
}

inline btQuaternion convertAPhyQuaternion(const aphy_quaternion &quaternion)
{
    return btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

inline btMatrix3x3 convertAPhyMatrix(const aphy_matrix3x3 &matrix)
{
    return btMatrix3x3(
        matrix.firstRow.x, matrix.firstRow.y, matrix.firstRow.z,
        matrix.secondRow.x, matrix.secondRow.y, matrix.secondRow.z,
        matrix.thirdRow.x, matrix.thirdRow.y, matrix.thirdRow.z);
}

inline btTransform convertAPhyTransform(const aphy_transform &transform)
{
    return btTransform(convertAPhyMatrix(transform.rotation), convertAPhyVector(transform.origin));
}

#endif //APHY_UTILITY_HPP
