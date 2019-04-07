
#ifndef APHY_HPP_
#define APHY_HPP_

#include "aphy.h"
#include <stdexcept>
#include <memory>
#include <atomic>

namespace aphy
{

extern aphy_icd_dispatch cppRefcountedDispatchTable;

/**
 * Phanapi reference counter
 */
template <typename T>
class ref_counter
{
public:
    ref_counter(T *cobject)
        : dispatchTable(&cppRefcountedDispatchTable), object(cobject), strongCount(1), weakCount(0)
    {
    }

    aphy_error retain()
    {
        // Check once before doing the increase.
        if(strongCount == 0)
            return APHY_INVALID_OPERATION;

        // Increase the referenece count.
        auto old = strongCount.fetch_add(1, std::memory_order_relaxed);

        // Check again, for concurrency reasons.
        if(old == 0)
            return APHY_INVALID_OPERATION;

        return APHY_OK;
    }

    aphy_error release()
    {
        // First sanity check.
        if(strongCount == 0)
            return APHY_INVALID_OPERATION;

        // Decrease the strong count.
        auto old = strongCount.fetch_sub(1, std::memory_order_relaxed);

        // Check again, for concurrency reasons.
        if(old == 0)
            return APHY_INVALID_OPERATION;

        // Should I delete the object?
        if(old == 1)
        {
            delete object;

            // Should I delete myself?
            if(weakCount == 0)
                delete this;
        }

        return APHY_OK;
    }

    bool weakLock()
    {
        unsigned int oldCount;
        while((oldCount = strongCount.load()) != 0)
        {
            if(strongCount.compare_exchange_weak(oldCount, oldCount + 1))
                return true;
        }

        return false;
    }

    void weakRetain()
    {
    }

    void weakRelease()
    {
    }

    aphy_icd_dispatch *dispatchTable;
    T * object;
    std::atomic_uint strongCount;
    std::atomic_uint weakCount;
};

template<typename T>
class weak_ref;

/**
 * Phanapi strong reference
 */
template<typename T>
class ref
{
public:
    typedef ref_counter<T> Counter;
    typedef ref<T> StrongRef;
    typedef weak_ref<T> WeakRef;

private:
    Counter *counter;
    friend WeakRef;

public:
    ref() : counter(nullptr) {}

    ref(const StrongRef &other)
        : counter(nullptr)
    {
        *this = other;
    }

    explicit ref(Counter *theCounter)
        : counter(theCounter)
    {
    }

    ~ref()
    {
        if(counter)
            counter->release();
    }

    StrongRef &operator=(const StrongRef &other)
    {
        auto newCounter = other.counter;
        if(newCounter)
            newCounter->retain();
        if(counter)
            counter->release();
        counter = newCounter;
        return *this;
    }

    void reset(Counter *newCounter = nullptr)
    {
        if(counter)
            counter->release();
        counter = newCounter;
    }

    Counter *disown()
    {
        Counter *result = counter;
        counter = nullptr;
        return result;
    }

    template<typename U>
    U *as() const
    {
        return static_cast<U*> (counter->object);
    }

    T *get() const
    {
        return counter->object; 
    }

    T *operator->() const
    {
        return get();
    }

    operator bool() const
    {
        return counter != nullptr;
    }

    bool operator==(const StrongRef &other) const
    {
        return counter == other.counter;
    }

    bool operator<(const StrongRef &other) const
    {
        return counter < other.counter;
    }

    size_t hash() const
    {
        return std::hash<Counter*> () (counter);
    }
};

/**
 * Phanapi weak reference
 */
template<typename T>
class weak_ref
{
public:
    typedef ref_counter<T> Counter;
    typedef ref<T> StrongRef;
    typedef weak_ref<T> WeakRef;

private:
    Counter *counter;

public:
    explicit weak_ref(const StrongRef &ref)
    {
        counter = ref.counter;
        if(counter)
            counter->weakRetain();
    }

    weak_ref(const WeakRef &ref)
    {
        counter = ref.counter;
        if(counter)
            counter->weakRetain();
    }

    ~weak_ref()
    {
        if(counter)
            counter->weakRelease();
    }

    WeakRef &operator=(const StrongRef &other)
    {
        auto newCounter = other.counter;
        if(newCounter)
            newCounter->weakRetain();
        if(counter)
            counter->weakRelease();
        counter = newCounter;
        return *this;
    }

    WeakRef &operator=(const WeakRef &other)
    {
        auto newCounter = other.counter;
        if(newCounter)
            newCounter->weakRetain();
        if(counter)
            counter->weakRelease();
        counter = newCounter;
        return *this;
    }

    StrongRef lock()
    {
        if(!counter)
            return StrongRef();

        return counter->weakLock() ? StrongRef(counter) : StrongRef();
    }

    bool operator==(const WeakRef &other) const
    {
        return counter == other.counter;
    }

    bool operator<(const WeakRef &other) const
    {
        return counter < other.counter;
    }

    size_t hash() const
    {
        return std::hash<Counter*> () (counter);
    }
};

template<typename I, typename T, typename...Args>
inline ref<I> makeObjectWithInterface(Args... args)
{
    std::unique_ptr<T> object(new T(args...));
    std::unique_ptr<ref_counter<I>> counter(new ref_counter<I> (object.release()));
    return ref<I> (counter.release());
}

template<typename T, typename...Args>
inline ref<typename T::main_interface> makeObject(Args... args)
{
   return makeObjectWithInterface<typename T::main_interface, T> (args...);
}

/**
 * Phanapi base interface
 */
class base_interface
{
public:
    virtual ~base_interface() {}
};

} // End of namespace aphy

namespace std
{
template<typename T>
struct hash<aphy::ref<T>>
{
    size_t operator()(const aphy::ref<T> &ref) const
    {
        return ref.hash();
    }
};

template<typename T>
struct hash<aphy::weak_ref<T>>
{
    size_t operator()(const aphy::ref<T> &ref) const
    {
        return ref.hash();
    }
};

}

namespace aphy
{
struct engine;
typedef ref_counter<engine> *engine_ptr;
typedef ref<engine> engine_ref;
typedef weak_ref<engine> engine_weakref;

struct collision_configuration;
typedef ref_counter<collision_configuration> *collision_configuration_ptr;
typedef ref<collision_configuration> collision_configuration_ref;
typedef weak_ref<collision_configuration> collision_configuration_weakref;

struct collision_dispatcher;
typedef ref_counter<collision_dispatcher> *collision_dispatcher_ptr;
typedef ref<collision_dispatcher> collision_dispatcher_ref;
typedef weak_ref<collision_dispatcher> collision_dispatcher_weakref;

struct broadphase;
typedef ref_counter<broadphase> *broadphase_ptr;
typedef ref<broadphase> broadphase_ref;
typedef weak_ref<broadphase> broadphase_weakref;

struct constraint_solver;
typedef ref_counter<constraint_solver> *constraint_solver_ptr;
typedef ref<constraint_solver> constraint_solver_ref;
typedef weak_ref<constraint_solver> constraint_solver_weakref;

struct world;
typedef ref_counter<world> *world_ptr;
typedef ref<world> world_ref;
typedef weak_ref<world> world_weakref;

struct character_controller;
typedef ref_counter<character_controller> *character_controller_ptr;
typedef ref<character_controller> character_controller_ref;
typedef weak_ref<character_controller> character_controller_weakref;

struct collision_object;
typedef ref_counter<collision_object> *collision_object_ptr;
typedef ref<collision_object> collision_object_ref;
typedef weak_ref<collision_object> collision_object_weakref;

struct collision_shape;
typedef ref_counter<collision_shape> *collision_shape_ptr;
typedef ref<collision_shape> collision_shape_ref;
typedef weak_ref<collision_shape> collision_shape_weakref;

struct motion_state;
typedef ref_counter<motion_state> *motion_state_ptr;
typedef ref<motion_state> motion_state_ref;
typedef weak_ref<motion_state> motion_state_weakref;

// Interface wrapper for aphy_engine.
struct engine : base_interface
{
public:
	typedef engine main_interface;
	virtual aphy_cstring getName() = 0;
	virtual aphy_int getVersion() = 0;
	virtual collision_configuration_ptr createDefaultCollisionConfiguration() = 0;
	virtual collision_dispatcher_ptr createDefaultCollisionDispatcher(const collision_configuration_ref & collision_configuration) = 0;
	virtual broadphase_ptr createDefaultBroadphase() = 0;
	virtual constraint_solver_ptr createDefaultConstraintSolver() = 0;
	virtual motion_state_ptr createDefaultMotionState() = 0;
	virtual world_ptr createDynamicsWorld(const collision_dispatcher_ref & collision_dispatcher, const broadphase_ref & broadphase, const constraint_solver_ref & constraint_solver, const collision_configuration_ref & collision_configuration) = 0;
	virtual collision_shape_ptr createBoxShape(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) = 0;
	virtual collision_shape_ptr createCylinderX(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) = 0;
	virtual collision_shape_ptr createCylinderY(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) = 0;
	virtual collision_shape_ptr createCylinderZ(aphy_scalar half_width, aphy_scalar half_height, aphy_scalar half_depth) = 0;
	virtual collision_shape_ptr createConvexHullShape(aphy_scalar* coordinates, aphy_size num_points, aphy_size stride) = 0;
	virtual collision_shape_ptr createCapsuleShapeX(aphy_scalar radius, aphy_scalar height) = 0;
	virtual collision_shape_ptr createCapsuleShapeY(aphy_scalar radius, aphy_scalar height) = 0;
	virtual collision_shape_ptr createCapsuleShapeZ(aphy_scalar radius, aphy_scalar height) = 0;
	virtual collision_shape_ptr createCompoundShape() = 0;
	virtual collision_shape_ptr createConeX(aphy_scalar radius, aphy_scalar height) = 0;
	virtual collision_shape_ptr createConeY(aphy_scalar radius, aphy_scalar height) = 0;
	virtual collision_shape_ptr createConeZ(aphy_scalar radius, aphy_scalar height) = 0;
	virtual collision_shape_ptr createEmptyShape() = 0;
	virtual collision_shape_ptr createHeightfieldTerrainShape(aphy_int height_stick_width, aphy_int height_stick_length, aphy_pointer heightfield_data, aphy_scalar height_scale, aphy_scalar min_height, aphy_scalar max_height, aphy_axis up_axis, aphy_scalar_type height_data_type, aphy_bool flip_quad_edges, aphy_scalar local_scale_x, aphy_scalar local_scale_y, aphy_scalar local_scale_z) = 0;
	virtual collision_shape_ptr createSphere(aphy_scalar radius) = 0;
	virtual collision_object_ptr createSimpleRigidBody(aphy_scalar mass, const motion_state_ref & motion_state, const collision_shape_ref & collision_shape, aphy_vector3 local_inertia) = 0;
	virtual collision_object_ptr createSimpleRigidBodyFrom(aphy_scalar mass, const motion_state_ref & motion_state, const collision_shape_ref & collision_shape, aphy_vector3* local_inertia) = 0;
	virtual collision_object_ptr createGhostObject() = 0;
	virtual collision_object_ptr createPairCachingGhostObject() = 0;
	virtual character_controller_ptr createKinematicCharacterController(const collision_object_ref & ghost_object, const collision_shape_ref & convex_shape, aphy_scalar step_height, aphy_axis up_axis) = 0;
};


// Interface wrapper for aphy_collision_configuration.
struct collision_configuration : base_interface
{
public:
	typedef collision_configuration main_interface;
};


// Interface wrapper for aphy_collision_dispatcher.
struct collision_dispatcher : base_interface
{
public:
	typedef collision_dispatcher main_interface;
};


// Interface wrapper for aphy_broadphase.
struct broadphase : base_interface
{
public:
	typedef broadphase main_interface;
};


// Interface wrapper for aphy_constraint_solver.
struct constraint_solver : base_interface
{
public:
	typedef constraint_solver main_interface;
};


// Interface wrapper for aphy_world.
struct world : base_interface
{
public:
	typedef world main_interface;
	virtual aphy_uint getNumberOfCollisionObject() = 0;
	virtual aphy_uint getNumberOfConstraints() = 0;
	virtual aphy_error addCollisionObject(const collision_object_ref & object, aphy_short collision_filter_group, aphy_short collision_filter_mask) = 0;
	virtual aphy_error removeCollisionObject(const collision_object_ref & object) = 0;
	virtual aphy_error addRigidBody(const collision_object_ref & object) = 0;
	virtual aphy_error removeRigidBody(const collision_object_ref & object) = 0;
	virtual aphy_error addCharacterController(const character_controller_ref & character) = 0;
	virtual aphy_error removeCharacterController(const character_controller_ref & character) = 0;
	virtual aphy_error addRigidBodyWithFilter(const collision_object_ref & object, aphy_short collision_filter_group, aphy_short collision_filter_mask) = 0;
	virtual aphy_error stepSimulation(aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step) = 0;
	virtual aphy_error setGravity(aphy_scalar x, aphy_scalar y, aphy_scalar z) = 0;
	virtual aphy_size encodeDebugDrawing() = 0;
	virtual aphy_error getDebugDrawingData(aphy_size buffer_size, aphy_pointer buffer) = 0;
};


// Interface wrapper for aphy_character_controller.
struct character_controller : base_interface
{
public:
	typedef character_controller main_interface;
	virtual aphy_error setWalkDirection(aphy_vector3 direction) = 0;
	virtual aphy_error setWalkDirectionFrom(aphy_vector3* direction) = 0;
	virtual aphy_error setVelocityForTimeInterval(aphy_vector3 velocity, aphy_scalar time_interval) = 0;
	virtual aphy_error setVelocityForTimeIntervalFrom(aphy_vector3* velocity, aphy_scalar time_interval) = 0;
	virtual aphy_error warp(aphy_vector3 origin) = 0;
	virtual aphy_error warpWithOriginFrom(aphy_vector3* origin) = 0;
	virtual aphy_bool canJump() = 0;
	virtual aphy_error jump() = 0;
	virtual aphy_bool isOnGround() = 0;
	virtual aphy_error setMaxJumpHeight(aphy_scalar height) = 0;
	virtual aphy_error setJumpSpeed(aphy_scalar speed) = 0;
	virtual aphy_error setGravity(aphy_scalar gravity) = 0;
};


// Interface wrapper for aphy_collision_object.
struct collision_object : base_interface
{
public:
	typedef collision_object main_interface;
	virtual aphy_transform getTransform() = 0;
	virtual aphy_error getTransformInto(aphy_transform* result) = 0;
	virtual aphy_vector3 getTranslation() = 0;
	virtual aphy_error getTranslationInto(aphy_vector3* result) = 0;
	virtual aphy_matrix3x3 getMatrix() = 0;
	virtual aphy_error getMatrixInto(aphy_matrix3x3* result) = 0;
	virtual aphy_quaternion getQuaternion() = 0;
	virtual aphy_error getQuaternionInto(aphy_quaternion* result) = 0;
	virtual aphy_error setTransform(aphy_transform value) = 0;
	virtual aphy_error setTransformFrom(aphy_transform* value) = 0;
	virtual aphy_error setTranslation(aphy_vector3 value) = 0;
	virtual aphy_error setTranslationFrom(aphy_vector3* value) = 0;
	virtual aphy_error setMatrix(aphy_matrix3x3 value) = 0;
	virtual aphy_error setMatrixFrom(aphy_matrix3x3* value) = 0;
	virtual aphy_error setQuaternion(aphy_quaternion value) = 0;
	virtual aphy_error setQuaternion(aphy_quaternion* value) = 0;
	virtual aphy_error setCollisionShape(const collision_shape_ref & shape) = 0;
};


// Interface wrapper for aphy_collision_shape.
struct collision_shape : base_interface
{
public:
	typedef collision_shape main_interface;
	virtual aphy_error setMargin(aphy_scalar margin) = 0;
	virtual aphy_scalar getMargin() = 0;
	virtual aphy_vector3 computeLocalInertia(aphy_scalar mass) = 0;
	virtual aphy_error computeLocalInertiaInto(aphy_scalar mass, aphy_vector3* result) = 0;
	virtual aphy_error addLocalShapeWithTransform(const collision_shape_ref & shape, aphy_transform transform) = 0;
	virtual aphy_error addLocalShapeWithTransformFrom(const collision_shape_ref & shape, aphy_transform* transform) = 0;
};


// Interface wrapper for aphy_motion_state.
struct motion_state : base_interface
{
public:
	typedef motion_state main_interface;
	virtual aphy_transform getTransform() = 0;
	virtual aphy_error getTransformInto(aphy_transform* result) = 0;
	virtual aphy_vector3 getTranslation() = 0;
	virtual aphy_error getTranslationInto(aphy_vector3* result) = 0;
	virtual aphy_matrix3x3 getMatrix() = 0;
	virtual aphy_error getMatrixInto(aphy_matrix3x3* result) = 0;
	virtual aphy_quaternion getQuaternion() = 0;
	virtual aphy_error getQuaternionInto(aphy_quaternion* result) = 0;
	virtual aphy_error setTransform(aphy_transform value) = 0;
	virtual aphy_error setTransformFrom(aphy_transform* value) = 0;
	virtual aphy_error setTranslation(aphy_vector3 value) = 0;
	virtual aphy_error setTranslationFrom(aphy_vector3* value) = 0;
	virtual aphy_error setMatrix(aphy_matrix3x3 value) = 0;
	virtual aphy_error setMatrixFrom(aphy_matrix3x3* value) = 0;
	virtual aphy_error setQuaternion(aphy_quaternion value) = 0;
	virtual aphy_error setQuaternionFrom(aphy_quaternion* value) = 0;
};


} // End of aphy

#endif /* APHY_HPP_ */
