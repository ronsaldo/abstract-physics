#include "Broadphase.hpp"
#include "CollisionConfiguration.hpp"
#include "CollisionDispatcher.hpp"
#include "CollisionObject.hpp"
#include "ConstraintSolver.hpp"
#include "CharacterController.hpp"
#include "World.hpp"
#include <vector>

namespace APhyBullet
{

class BulletWorldDebugDrawer : public btIDebugDraw
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

    void drawLine(const btVector3& from,const btVector3& to,const btVector3& color) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_LINE);
        putVector3(from);
        putVector3(to);
        putVector3(color);
    }

	void drawLine(const btVector3& from,const btVector3& to, const btVector3& fromColor, const btVector3& toColor) override
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

    void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) override
    {
        putOpcode(APHY_DEBUG_DRAW_OP_CONTACT_POINT);
        putVector3(PointOnB);
        putVector3(normalOnB);
        putScalar(distance);
        putInt32(lifeTime);
        putVector3(color);
    }

    void reportErrorWarning(const char* warningString) override
    {

    }

    void draw3dText(const btVector3& location,const char* textString) override
    {

    }

    void setDebugMode(int debugMode) override
    {
        currentDebugMode = debugMode;
    }

    int	getDebugMode() const override
    {
        return currentDebugMode;
    }


    int currentDebugMode = 0;
    std::vector<uint8_t> instructions;
};

BulletWorld::BulletWorld(btDynamicsWorld *handle)
    : handle(handle)
{
}

BulletWorld::~BulletWorld()
{
    delete handle;
}

aphy_uint BulletWorld::getNumberOfCollisionObject()
{
    return handle->getNumCollisionObjects();
}

aphy_uint BulletWorld::getNumberOfConstraints()
{
    return handle->getNumConstraints();
}

aphy_error BulletWorld::addCollisionObject(const collision_object_ref &object, aphy_short collision_filter_group, aphy_short collision_filter_mask)
{
    CHECK_POINTER(object);
    handle->addCollisionObject(object.as<BulletCollisionObject>()->handle, collision_filter_group, collision_filter_mask);

    collisionObjects.insert(object);
    return APHY_OK;
}

aphy_error BulletWorld::removeCollisionObject(const collision_object_ref &object)
{
    CHECK_POINTER(object);
    handle->removeCollisionObject(object.as<BulletCollisionObject>()->handle);

    auto it = collisionObjects.find(object);
    if(it != collisionObjects.end())
        collisionObjects.erase(it);

    return APHY_OK;
}

aphy_error BulletWorld::addRigidBody(const collision_object_ref &object)
{
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object.as<BulletCollisionObject>()->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    handle->addRigidBody(rigidBody);

    collisionObjects.insert(object);
    return APHY_OK;
}

aphy_error BulletWorld::removeRigidBody(const collision_object_ref &object)
{
    auto rigidBody = btRigidBody::upcast(object.as<BulletCollisionObject>()->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    handle->removeRigidBody(rigidBody);

    auto it = collisionObjects.find(object);
    if(it != collisionObjects.end())
        collisionObjects.erase(it);

    return APHY_OK;
}

aphy_error BulletWorld::addCharacterController(const character_controller_ref &character)
{
    CHECK_POINTER(character);

    handle->addAction(character.as<BulletCharacterController>()->handle);
    characterControllers.insert(character);
    return APHY_OK;
}

aphy_error BulletWorld::removeCharacterController(const character_controller_ref &character)
{
    CHECK_POINTER(character);

    handle->removeAction(character.as<BulletCharacterController>()->handle);
    auto it = characterControllers.find(character);
    if(it != characterControllers.end())
        characterControllers.erase(it);
    return APHY_OK;
}

aphy_error BulletWorld::addRigidBodyWithFilter(const collision_object_ref &object, aphy_short collision_filter_group, aphy_short collision_filter_mask)
{
    CHECK_POINTER(object);

    auto rigidBody = btRigidBody::upcast(object.as<BulletCollisionObject>()->handle);
    if(!rigidBody)
        return APHY_INVALID_PARAMETER;

    handle->addRigidBody(rigidBody, collision_filter_group, collision_filter_mask);

    collisionObjects.insert(object);
    return APHY_OK;
}

aphy_error BulletWorld::stepSimulation(aphy_scalar time_step, aphy_int max_sub_steps, aphy_scalar fixed_time_step)
{
    handle->stepSimulation(time_step, max_sub_steps, fixed_time_step);
    return APHY_OK;
}

aphy_error BulletWorld::setGravity(aphy_scalar x, aphy_scalar y, aphy_scalar z)
{
    handle->setGravity(btVector3(x, y, z));
    return APHY_OK;
}

aphy_size BulletWorld::encodeDebugDrawing()
{
    if(!debugDrawer)
    {
        debugDrawer = std::make_shared<BulletWorldDebugDrawer> ();
        handle->setDebugDrawer(debugDrawer.get());
    }

    debugDrawer->reset();
    debugDrawer->setDebugMode(/*btIDebugDraw::DBG_DrawWireframe |*/ btIDebugDraw::DBG_DrawAabb);
    handle->debugDrawWorld();
    return aphy_size(debugDrawer->instructions.size());
}

aphy_error BulletWorld::getDebugDrawingData(aphy_size buffer_size, aphy_pointer buffer )
{
    CHECK_POINTER(buffer);

    if(!debugDrawer || buffer_size < debugDrawer->instructions.size())
        return APHY_INVALID_OPERATION;

    memcpy(buffer, debugDrawer->instructions.data(), debugDrawer->instructions.size());
    return APHY_OK;
}

} // End of namespace APhyBullet
