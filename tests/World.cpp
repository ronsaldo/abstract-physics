#include <APHY/aphy.hpp>

#include <UnitTest++.h>

SUITE(Engine)
{
    class WorldFixture
    {
    public:
        WorldFixture()
        {
            aphy_size numEngines;
            aphyGetEngines(1, (aphy_engine**)&engine, &numEngines);
            CHECK(numEngines > 0);

            collisionConfiguration = engine->createDefaultCollisionConfiguration();
            collisionDispatcher = engine->createDefaultCollisionDispatcher(collisionConfiguration.get());
            broadphase = engine->createDefaultBroadphase();
            constraintSolver = engine->createDefaultConstraintSolver();
            world = engine->createDynamicsWorld(collisionDispatcher.get(), broadphase.get(), constraintSolver.get(), collisionConfiguration.get());
        }

        aphy_engine_ref engine;

        aphy_collision_configuration_ref collisionConfiguration;
        aphy_collision_dispatcher_ref collisionDispatcher;
        aphy_broadphase_ref broadphase;
        aphy_constraint_solver_ref constraintSolver;
        aphy_world_ref world;
    };

    TEST_FIXTURE(WorldFixture, EmptyWorld)
    {
        CHECK_EQUAL(0u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());
    }

    TEST_FIXTURE(WorldFixture, StepEmptyWorld)
    {
        CHECK_EQUAL(0u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());
        world->stepSimulation(1.0, 10, 1.0/60.0);
        CHECK_EQUAL(0u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());
    }
}
