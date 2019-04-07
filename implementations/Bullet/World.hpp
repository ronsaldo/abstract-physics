#ifndef APHY_WORLD_HPP_
#define APHY_WORLD_HPP_

#include <unordered_set>
#include "Common.hpp"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include <memory>

namespace APhyBullet
{

class BulletWorldDebugDrawer;

/**
* Bullet world
*/
struct BulletWorld : public aphy::world
{
public:
    BulletWorld(btDynamicsWorld *handle);
    ~BulletWorld();

    virtual aphy_uint getNumberOfCollisionObject() override;
	virtual aphy_uint getNumberOfConstraints() override;
	virtual aphy_error addCollisionObject(const collision_object_ref &object, aphy_short collision_filter_group, aphy_short collision_filter_mask) override;
	virtual aphy_error removeCollisionObject(const collision_object_ref &object) override;
	virtual aphy_error addRigidBody(const collision_object_ref &object) override;
	virtual aphy_error removeRigidBody(const collision_object_ref &object) override;
	virtual aphy_error addCharacterController(const character_controller_ref &character) override;
	virtual aphy_error removeCharacterController(const character_controller_ref &character) override;
	virtual aphy_error addRigidBodyWithFilter(const collision_object_ref &object, aphy_short collision_filter_group, aphy_short collision_filter_mask) override;
	virtual aphy_error stepSimulation(aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step) override;
	virtual aphy_error setGravity(aphy_scalar x, aphy_scalar y, aphy_scalar z) override;
	virtual aphy_size encodeDebugDrawing() override;
	virtual aphy_error getDebugDrawingData(aphy_size buffer_size, aphy_pointer buffer) override;

    btDynamicsWorld *handle;
    collision_dispatcher_ref collisionDispatcher;
    broadphase_ref broadphase;
    constraint_solver_ref constraintSolver;
    collision_configuration_ref collisionConfiguration;

    std::unordered_set<collision_object_ref> collisionObjects;
    std::unordered_set<character_controller_ref> characterControllers;
    std::shared_ptr<BulletWorldDebugDrawer> debugDrawer;
};

} // End of namespace APhyBullet

#endif //APHY_WORLD_HPP_
