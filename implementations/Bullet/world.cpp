#include "broadphase.hpp"
#include "collision_configuration.hpp"
#include "collision_dispatcher.hpp"
#include "collision_object.hpp"
#include "constraint_solver.hpp"
#include "world.hpp"
#include <vector>

class aphy_bullet_world_debug_drawer : public btIDebugDraw
{
public:

    void reset()
    {
        instructions.clear();
    }
    
    void putOpcode(aphy_debug_draw_opcode opcode)
    {
        instructions.push_back(opcode);
    }

    void putInt32(int32_t value)
    {
        instructions.push_back(0);
        instructions.push_back(0);
        instructions.push_back(0);
        instructions.push_back(0);
        *reinterpret_cast<int32_t*> (&instructions[instructions.size() - 4]) = value;
    }
    
    void putScalar(float scalar)
    {
        instructions.push_back(0);
        instructions.push_back(0);
        instructions.push_back(0);
        instructions.push_back(0);
        *reinterpret_cast<float*> (&instructions[instructions.size() - 4]) = scalar;
    }
    
    void putVector3(const btVector3 &vector)
    {
        putScalar(vector.x());
        putScalar(vector.y());
        putScalar(vector.z());
    }
    
    virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& color) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_LINE);
        putVector3(from);
        putVector3(to);
        putVector3(color);
    }

	virtual void drawLine(const btVector3& from,const btVector3& to, const btVector3& fromColor, const btVector3& toColor) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_LINE_GRADIENT);
        putVector3(from);
        putVector3(to);
        putVector3(fromColor);
        putVector3(toColor);
    }

    virtual	void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& n0,const btVector3& n1,const btVector3& n2,const btVector3& color, btScalar alpha) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_TRIANGLE_LIGHTED);
        putVector3(v0);
        putVector3(v1);
        putVector3(v2);
        putVector3(n0);
        putVector3(n1);
        putVector3(n2);
        putVector3(color);
        putScalar(alpha);
    }

	virtual	void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& color, btScalar alpha) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_TRIANGLE_FLAT);
        putVector3(v0);
        putVector3(v1);
        putVector3(v2);
        putVector3(color);
        putScalar(alpha);
    }

    virtual void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_CONTACT_POINT);
        putVector3(PointOnB);
        putVector3(normalOnB);
        putScalar(distance);
        putInt32(lifeTime);
        putVector3(color);
    }

    virtual void reportErrorWarning(const char* warningString) override
    {
        
    }

    virtual void draw3dText(const btVector3& location,const char* textString) override
    {
        
    }
    
    virtual void setDebugMode(int debugMode) override
    {
        currentDebugMode = debugMode;
    }
    
    virtual int	getDebugMode() const override
    {
        return currentDebugMode;
    }

    
    int currentDebugMode = 0;
    std::vector<uint8_t> instructions;
};

_aphy_world::_aphy_world(btDynamicsWorld *handle)
    : handle(handle)
{
}

void _aphy_world::lostReferences()
{
    delete handle;

    if(collisionDispatcher)
        collisionDispatcher->release();
    if(broadphase)
        broadphase->release();
    if(constraintSolver)
        constraintSolver->release();
    if(collisionConfiguration)
        collisionConfiguration->release();

    for(auto obj : collisionObjects)
    {
        if(obj)
            obj->release();
    }
}

void _aphy_world::addedCollisionObject(aphy_collision_object *object)
{
    object->retain();
    collisionObjects.insert(object);
}

void _aphy_world::removedCollisionObject(aphy_collision_object *object)
{
    auto it = collisionObjects.find(object);
    if(it != collisionObjects.end())
    {
        object->release();
        collisionObjects.erase(it);
    }
}

aphy_size _aphy_world::encodeDebugDrawing()
{
    if(!debugDrawer)
    {
        debugDrawer = std::make_shared<aphy_bullet_world_debug_drawer> ();
        handle->setDebugDrawer(debugDrawer.get());
    }
    
    debugDrawer->reset();
    handle->debugDrawWorld();
    return aphy_size(debugDrawer->instructions.size());
}

aphy_error _aphy_world::getDebugDrawingData(aphy_size buffer_size, aphy_pointer buffer )
{
    CHECK_POINTER(buffer);
    
    if(!debugDrawer || buffer_size < debugDrawer->instructions.size())
        return APHY_INVALID_OPERATION;

    memcpy(buffer, debugDrawer->instructions.data(), debugDrawer->instructions.size());
    return APHY_OK;
}

// The exported C interface
APHY_EXPORT aphy_error aphyAddWorldReference ( aphy_world* world )
{
    CHECK_POINTER(world);
    return world->retain();
}

APHY_EXPORT aphy_error aphyReleaseWorldReference ( aphy_world* world )
{
    CHECK_POINTER(world);
    return world->release();
}

APHY_EXPORT aphy_uint aphyGetNumberOfCollisionObject ( aphy_world* world )
{
    if(!world)
        return 0;

    return world->handle->getNumCollisionObjects();
}

APHY_EXPORT aphy_uint aphyGetNumberOfConstraints ( aphy_world* world )
{
    if(!world)
        return 0;

    return world->handle->getNumConstraints();
}

APHY_EXPORT aphy_error aphyAddCollisionObject ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask )
{
    CHECK_POINTER(world);
    world->handle->addCollisionObject(object->handle, collision_filter_group, collision_filter_mask);
    world->addedCollisionObject(object);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyRemoveCollisionObject ( aphy_world* world, aphy_collision_object* object )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);
    world->handle->removeCollisionObject(object->handle);
    world->removedCollisionObject(object);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyAddRigidBody ( aphy_world* world, aphy_collision_object* object )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    world->handle->addRigidBody(rigidBody);
    world->addedCollisionObject(object);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyRemoveRigidBody ( aphy_world* world, aphy_collision_object* object )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    world->handle->removeRigidBody(rigidBody);
    world->removedCollisionObject(object);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyAddRigidBodyWithFilter ( aphy_world* world, aphy_collision_object* object, aphy_short collision_filter_group, aphy_short collision_filter_mask )
{
    CHECK_POINTER(world);
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    world->handle->addRigidBody(rigidBody, collision_filter_group, collision_filter_mask);
    world->addedCollisionObject(object);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphyStepSimulation ( aphy_world* world, aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step )
{
    CHECK_POINTER(world);
    world->handle->stepSimulation(time_step, max_sub_steps, fixed_time_step);
    return APHY_OK;
}

APHY_EXPORT aphy_error aphySetGravity ( aphy_world* world, aphy_scalar x, aphy_scalar y, aphy_scalar z )
{
    CHECK_POINTER(world);
    world->handle->setGravity(btVector3(x, y, z));
    return APHY_OK;
}

APHY_EXPORT aphy_size aphyEncodeDebugDrawing ( aphy_world* world )
{
    if(!world)
        return 0;
        
    return world->encodeDebugDrawing();
}

APHY_EXPORT aphy_error aphyGetDebugDrawingData ( aphy_world* world, aphy_size buffer_size, aphy_pointer buffer )
{
    CHECK_POINTER(world);
    return world->getDebugDrawingData(buffer_size, buffer);
}
