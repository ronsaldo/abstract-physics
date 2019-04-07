#ifndef APHY_CONSTRAINT_SOLVER_HPP_
#define APHY_CONSTRAINT_SOLVER_HPP_

#include "Common.hpp"
#include "btBulletDynamicsCommon.h"

namespace APhyBullet
{

/**
* Bullet constraint solver
*/
struct BulletConstraintSolver : public aphy::constraint_solver
{
public:
    BulletConstraintSolver(btConstraintSolver *handle);
    ~BulletConstraintSolver();

    btConstraintSolver *handle;
};

} // End of namespace APhyBullet

#endif //APHY_CONSTRAINT_SOLVER_HPP_
