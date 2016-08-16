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
            world->setGravity(0, -9.8, 0);
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

    TEST_FIXTURE(WorldFixture, FallingObject)
    {
        aphy_collision_shape_ref shape = engine->createBoxShape(0.5, 0.5, 0.5);
        aphy_motion_state_ref fallingObjectMotionState = engine->createDefaultMotionState();
        aphy_collision_object_ref fallingObject = engine->createSimpleRigidBody(1.0, fallingObjectMotionState.get(), shape.get(), shape->computeLocalInertia(1.0));
        world->addRigidBody(fallingObject.get());
        CHECK_EQUAL(1u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());

        auto initialPosition = fallingObjectMotionState->getTranslation();
        auto delta = 1.0 / 60.0;
        for(int i = 0; i < 10; ++i)
            world->stepSimulation(delta, 2, 1.0/60.0);

        auto newPosition = fallingObjectMotionState->getTranslation();
        CHECK(newPosition.y < initialPosition.y);
    }

    TEST_FIXTURE(WorldFixture, FallingObjectAndFloor)
    {
        // Floor
        aphy_collision_shape_ref floorShape = engine->createBoxShape(10.0, 0.2, 10.0);
        aphy_motion_state_ref floorMotionState = engine->createDefaultMotionState();
        aphy_collision_object_ref floorObject = engine->createSimpleRigidBody(0.0, floorMotionState.get(), floorShape.get(), {0});
        world->addRigidBody(floorObject.get());
        CHECK_EQUAL(1u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());

        // Falling object
        aphy_collision_shape_ref shape = engine->createBoxShape(0.5, 0.5, 0.5);
        aphy_motion_state_ref fallingObjectMotionState = engine->createDefaultMotionState();
        fallingObjectMotionState->setTranslation({0,2,0});
        aphy_collision_object_ref fallingObject = engine->createSimpleRigidBody(1.0, fallingObjectMotionState.get(), shape.get(), shape->computeLocalInertia(1.0));
        world->addRigidBody(fallingObject.get());
        CHECK_EQUAL(2u, world->getNumberOfCollisionObject());
        CHECK_EQUAL(0u, world->getNumberOfConstraints());

        auto initialPosition = fallingObjectMotionState->getTranslation();
        auto initialFloorPosition = floorMotionState->getTranslation();
        auto delta = 1.0 / 60.0;
        for(int i = 0; i < 30; ++i)
            world->stepSimulation(delta, 2, 1.0/60.0);

        auto newPosition = fallingObjectMotionState->getTranslation();
        auto newFloorPosition = floorMotionState->getTranslation();
        CHECK_EQUAL(initialFloorPosition.x, newFloorPosition.x);
        CHECK_EQUAL(initialFloorPosition.y, newFloorPosition.y);
        CHECK_EQUAL(initialFloorPosition.z, newFloorPosition.z);
        CHECK(newPosition.y < initialPosition.y);
        CHECK(newPosition.y > 0);
    }
}
