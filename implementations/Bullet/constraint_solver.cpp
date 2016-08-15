#include "constraint_solver.hpp"

_aphy_constraint_solver::_aphy_constraint_solver(btConstraintSolver *handle)
    : handle(handle)
{
}

void _aphy_constraint_solver::lostReferences()
{
    delete handle;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddConstraintSolverReference ( aphy_constraint_solver* constraint_solver )
{
    CHECK_POINTER(constraint_solver);
    return constraint_solver->retain();
}

APHY_EXPORT aphy_error aphyReleaseConstraintSolverReference ( aphy_constraint_solver* constraint_solver )
{
    CHECK_POINTER(constraint_solver);
    return constraint_solver->release();
}
