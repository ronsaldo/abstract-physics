#ifndef APHY_CONSTRAINT_SOLVER_HPP_
#define APHY_CONSTRAINT_SOLVER_HPP_

#include "object.hpp"
#include "btBulletDynamicsCommon.h"

/**
* Bullet constraint solver
*/
struct _aphy_constraint_solver : public Object<_aphy_constraint_solver>
{
public:
    _aphy_constraint_solver(btConstraintSolver *handle);

    void lostReferences();

    btConstraintSolver *handle;
};

#endif //APHY_CONSTRAINT_SOLVER_HPP_
