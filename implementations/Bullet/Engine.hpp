#ifndef APHY_ENGINE_HPP_
#define APHY_ENGINE_HPP_

#include "Common.hpp"

namespace APhyBullet
{

/**
* APhy bullet engine
*/
class BulletEngine : public aphy::engine
{
public:
    BulletEngine();
    ~BulletEngine();

    virtual aphy_cstring getName() override;
    virtual aphy_int getVersion() override;

    virtual collision_configuration_ptr createDefaultCollisionConfiguration() override;
	virtual collision_dispatcher_ptr createDefaultCollisionDispatcher(const collision_configuration_ref &collision_configuration) override;
	virtual broadphase_ptr createDefaultBroadphase() override;
	virtual constraint_solver_ptr createDefaultConstraintSolver() override;
	virtual motion_state_ptr createDefaultMotionState() override;
	virtual world_ptr createDynamicsWorld(const collision_dispatcher_ref &collision_dispatcher, const broadphase_ref &broadphase, const constraint_solver_ref &constraint_solver, const collision_configuration_ref &collision_configuration) override;
	virtual collision_shape_ptr createBoxShape(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) override;
	virtual collision_shape_ptr createCylinderX(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) override;
	virtual collision_shape_ptr createCylinderY(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) override;
	virtual collision_shape_ptr createCylinderZ(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) override;
	virtual collision_shape_ptr createConvexHullShape(aphy_scalar* coordinates, aphy_size num_points, aphy_size stride) override;
	virtual collision_shape_ptr createCapsuleShapeX(aphy_scalar radius, aphy_scalar height) override;
	virtual collision_shape_ptr createCapsuleShapeY(aphy_scalar radius, aphy_scalar height) override;
	virtual collision_shape_ptr createCapsuleShapeZ(aphy_scalar radius, aphy_scalar height) override;
	virtual collision_shape_ptr createCompoundShape() override;
	virtual collision_shape_ptr createConeX(aphy_scalar radius, aphy_scalar height) override;
	virtual collision_shape_ptr createConeY(aphy_scalar radius, aphy_scalar height) override;
	virtual collision_shape_ptr createConeZ(aphy_scalar radius, aphy_scalar height) override;
	virtual collision_shape_ptr createEmptyShape() override;
	virtual collision_shape_ptr createHeightfieldTerrainShape(aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data, aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis, aphy_scalar_type height_data_type, aphy_bool flip_quad_edges, aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z) override;
	virtual collision_shape_ptr createSphere(aphy_scalar radius) override;
	virtual collision_object_ptr createSimpleRigidBody(aphy_scalar mass, const motion_state_ref &motion_state, const collision_shape_ref &collision_shape, aphy_vector3 local_inertia) override;
	virtual collision_object_ptr createSimpleRigidBodyFrom(aphy_scalar mass, const motion_state_ref &motion_state, const collision_shape_ref &collision_shape, aphy_vector3* local_inertia) override;
	virtual collision_object_ptr createGhostObject() override;
	virtual collision_object_ptr createPairCachingGhostObject() override;
	virtual character_controller_ptr createKinematicCharacterController(const collision_object_ref &ghost_object, const collision_shape_ref &convex_shape, aphy_scalar step_height, aphy_axis up_axis) override;
};

}; // End of namespace APhyBullet;

#endif //APHY_ENGINE_HPP_
