#include <APHY/aphy.hpp>

#include <UnitTest++.h>

SUITE(Engine)
{
    TEST(CreateEngine)
    {
        aphy_size numEngines;
        aphy_engine_ref engine;
        aphyGetEngines(1, (aphy_engine**)&engine, &numEngines);
        CHECK(numEngines > 0);
    }

    TEST(CreateWorld)
    {
        aphy_size numEngines;
        aphy_engine_ref engine;
        aphyGetEngines(1, (aphy_engine**)&engine, &numEngines);
        CHECK(numEngines > 0);

        aphy_collision_configuration_ref collisionConfiguration = engine->createDefaultCollisionConfiguration();
        aphy_collision_dispatcher_ref collisionDispatcher = engine->createDefaultCollisionDispatcher(collisionConfiguration.get());
        aphy_broadphase_ref broadphase = engine->createDefaultBroadphase();
        aphy_constraint_solver_ref constraintSolver = engine->createDefaultConstraintSolver();
        aphy_world_ref world = engine->createDynamicsWorld(collisionDispatcher.get(), broadphase.get(), constraintSolver.get(), collisionConfiguration.get());
        CHECK_EQUAL(0u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());
    }
}
