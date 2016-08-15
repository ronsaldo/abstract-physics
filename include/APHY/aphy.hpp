
#ifndef APHY_HPP_
#define APHY_HPP_

#include <stdexcept>
#include "APHY/aphy.h"

/**
 * Abstract Physics exception.
 */
class aphy_exception : public std::runtime_error
{
public:
    explicit aphy_exception(aphy_error error)
        : std::runtime_error("AGPU Error"), errorCode(error)
    {
    }

    aphy_error getErrorCode() const
    {
        return errorCode;
    }

private:
    aphy_error errorCode;
};

/**
 * Abstract GPU reference smart pointer.
 */
template<typename T>
class aphy_ref
{
public:
    aphy_ref()
        : pointer(0)
    {
    }

    aphy_ref(const aphy_ref<T> &other)
    {
        if(other.pointer)
            other.pointer->addReference();
        pointer = other.pointer;
    }

    aphy_ref(T* pointer)
        : pointer(pointer)
    {
    }

    ~aphy_ref()
    {
        if (pointer)
            pointer->release();
    }

    aphy_ref<T> &operator=(T *newPointer)
    {
        if (pointer)
            pointer->release();
        pointer = newPointer;
        return *this;
    }

    aphy_ref<T> &operator=(const aphy_ref<T> &other)
    {
        if(pointer != other.pointer)
        {
            if(other.pointer)
                other.pointer->addReference();
            if(pointer)
                pointer->release();
            pointer = other.pointer;
        }
        return *this;
    }

    operator bool() const
    {
        return pointer;
    }

    bool operator!() const
    {
        return !pointer;
    }

    T* get() const
    {
        return pointer;
    }

    T *operator->() const
    {
        return pointer;
    }

private:
    T *pointer;
};

/**
 * Helper function to convert an error code into an exception.
 */
inline void APhyThrowIfFailed(aphy_error error)
{
    if(error < 0)
        throw aphy_exception(error);
}

// Interface wrapper for aphy_engine.
struct _aphy_engine
{
private:
	_aphy_engine() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddEngineReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseEngine( this ));
	}

	inline aphy_cstring getName (  )
	{
		return aphyGetEngineName( this );
	}

	inline aphy_int getVersion (  )
	{
		return aphyGetEngineVersion( this );
	}

	inline aphy_collision_configuration* createDefaultCollisionConfiguration (  )
	{
		return aphyCreateDefaultCollisionConfiguration( this );
	}

	inline aphy_collision_dispatcher* createDefaultCollisionDispatcher ( aphy_collision_configuration* collision_configuration )
	{
		return aphyCreateDefaultCollisionDispatcher( this, collision_configuration );
	}

	inline aphy_broadphase* createDefaultBroadphase (  )
	{
		return aphyCreateDefaultBroadphase( this );
	}

	inline aphy_constraint_solver* createDefaultConstraintSolver (  )
	{
		return aphyCreateDefaultConstraintSolver( this );
	}

	inline aphy_world* createDynamicsWorld ( aphy_collision_dispatcher* collision_dispatcher, aphy_broadphase* broadphase, aphy_constraint_solver* constraint_solver, aphy_collision_configuration* collision_configuration )
	{
		return aphyCreateDynamicsWorld( this, collision_dispatcher, broadphase, constraint_solver, collision_configuration );
	}

	inline aphy_collision_shape* createBoxShape ( aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
	{
		return aphyCreateBoxShape( this, half_width, half_height, half_depth );
	}

	inline aphy_collision_shape* createCylinderX ( aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
	{
		return aphyCreateCylinderX( this, half_width, half_height, half_depth );
	}

	inline aphy_collision_shape* createCylinderY ( aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
	{
		return aphyCreateCylinderY( this, half_width, half_height, half_depth );
	}

	inline aphy_collision_shape* createCylinderZ ( aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth )
	{
		return aphyCreateCylinderZ( this, half_width, half_height, half_depth );
	}

	inline aphy_collision_shape* createConvexHullShape ( aphy_scalar* coordinates, aphy_size num_points, aphy_size stride )
	{
		return aphyCreateConvexHullShape( this, coordinates, num_points, stride );
	}

	inline aphy_collision_shape* createCapsuleShapeX ( aphy_scalar radius, aphy_scalar height )
	{
		return aphyCreateCapsuleShapeX( this, radius, height );
	}

	inline aphy_collision_shape* createCapsuleShapeY ( aphy_scalar radius, aphy_scalar height )
	{
		return aphyCreateCapsuleShapeY( this, radius, height );
	}

	inline aphy_collision_shape* createCapsuleShapeZ ( aphy_scalar radius, aphy_scalar height )
	{
		return aphyCreateCapsuleShapeZ( this, radius, height );
	}

	inline aphy_collision_shape* createConeX ( aphy_scalar radius, aphy_scalar height )
	{
		return aphyCreateConeX( this, radius, height );
	}

	inline aphy_collision_shape* createConeY ( aphy_scalar radius, aphy_scalar height )
	{
		return aphyCreateConeY( this, radius, height );
	}

	inline aphy_collision_shape* createConeZ ( aphy_scalar radius, aphy_scalar height )
	{
		return aphyCreateConeZ( this, radius, height );
	}

	inline aphy_collision_shape* createSphere ( aphy_scalar radius )
	{
		return aphyCreateSphere( this, radius );
	}

};

typedef aphy_ref<aphy_engine> aphy_engine_ref;

// Interface wrapper for aphy_collision_configuration.
struct _aphy_collision_configuration
{
private:
	_aphy_collision_configuration() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddCollisionConfigurationReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseCollisionConfiguration( this ));
	}

};

typedef aphy_ref<aphy_collision_configuration> aphy_collision_configuration_ref;

// Interface wrapper for aphy_collision_dispatcher.
struct _aphy_collision_dispatcher
{
private:
	_aphy_collision_dispatcher() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddCollisionDispatcherReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseCollisionDispatcher( this ));
	}

};

typedef aphy_ref<aphy_collision_dispatcher> aphy_collision_dispatcher_ref;

// Interface wrapper for aphy_broadphase.
struct _aphy_broadphase
{
private:
	_aphy_broadphase() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddBroadphaseReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseBroadphaseReference( this ));
	}

};

typedef aphy_ref<aphy_broadphase> aphy_broadphase_ref;

// Interface wrapper for aphy_constraint_solver.
struct _aphy_constraint_solver
{
private:
	_aphy_constraint_solver() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddConstraintSolverReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseConstraintSolverReference( this ));
	}

};

typedef aphy_ref<aphy_constraint_solver> aphy_constraint_solver_ref;

// Interface wrapper for aphy_world.
struct _aphy_world
{
private:
	_aphy_world() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddWorldReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseWorldReference( this ));
	}

	inline aphy_uint getNumberOfCollisionObject (  )
	{
		return aphyGetNumberOfCollisionObject( this );
	}

	inline aphy_uint getNumberOfConstraints (  )
	{
		return aphyGetNumberOfConstraints( this );
	}

	inline void addCollisionObject ( aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask )
	{
		APhyThrowIfFailed(aphyAddCollisionObject( this, object, collision_filter_group, collision_filter_mask ));
	}

	inline void removeCollisionObject ( aphy_collision_object* object )
	{
		APhyThrowIfFailed(aphyRemoveCollisionObject( this, object ));
	}

	inline void addRigidBody ( aphy_collision_object* object )
	{
		APhyThrowIfFailed(aphyAddRigidBody( this, object ));
	}

	inline void removeRigidBody ( aphy_collision_object* object )
	{
		APhyThrowIfFailed(aphyRemoveRigidBody( this, object ));
	}

	inline void addRigidBodyWithFilter ( aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask )
	{
		APhyThrowIfFailed(aphyAddRigidBodyWithFilter( this, object, collision_filter_group, collision_filter_mask ));
	}

	inline void stepSimulation ( aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step )
	{
		APhyThrowIfFailed(aphyStepSimulation( this, time_step, max_sub_steps, fixed_time_step ));
	}

	inline void setGravity ( aphy_scalar x, aphy_scalar y, aphy_scalar z )
	{
		APhyThrowIfFailed(aphySetGravity( this, x, y, z ));
	}

};

typedef aphy_ref<aphy_world> aphy_world_ref;

// Interface wrapper for aphy_collision_object.
struct _aphy_collision_object
{
private:
	_aphy_collision_object() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddCollisionObjectReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseCollisionObjectReference( this ));
	}

};

typedef aphy_ref<aphy_collision_object> aphy_collision_object_ref;

// Interface wrapper for aphy_collision_shape.
struct _aphy_collision_shape
{
private:
	_aphy_collision_shape() {}

public:
	inline void addReference (  )
	{
		APhyThrowIfFailed(aphyAddCollisionShapeReference( this ));
	}

	inline void release (  )
	{
		APhyThrowIfFailed(aphyReleaseCollisionShapeReference( this ));
	}

	inline void setMargin ( aphy_scalar margin )
	{
		APhyThrowIfFailed(aphySetShapeMargin( this, margin ));
	}

	inline aphy_scalar getMargin (  )
	{
		return aphyGetShapeMargin( this );
	}

};

typedef aphy_ref<aphy_collision_shape> aphy_collision_shape_ref;


#endif /* APHY_HPP_ */