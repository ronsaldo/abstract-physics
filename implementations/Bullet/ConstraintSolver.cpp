#include "ConstraintSolver.hpp"

namespace APhyBullet
{

BulletConstraintSolver::BulletConstraintSolver(btConstraintSolver *handle)
    : handle(handle)
{
}

BulletConstraintSolver::~BulletConstraintSolver()
{
    delete handle;
}

} // End of namespace APhyBullet
