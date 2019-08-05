option(BUILD_SHARED_LIBS "" OFF)
option(BUILD_UNIT_TESTS "" OFF)
option(BUILD_CPU_DEMOS "" OFF)
option(BUILD_BULLET3 "" OFF)
option(BUILD_BULLET2_DEMOS "" OFF)
option(BUILD_EXTRAS "" OFF)
option(USE_GLUT "" OFF)
option(USE_GRAPHICAL_BENCHMARK "" OFF)
OPTION(INSTALL_LIBS "" OFF)
add_subdirectory(bullet3)

set(APHY_Bullet_SOURCES
    Broadphase.cpp
    Broadphase.hpp
    CollisionConfiguration.cpp
    CollisionConfiguration.hpp
    CollisionDispatcher.cpp
    CollisionDispatcher.hpp
    CollisionMeshCollection.cpp
    CollisionMeshCollection.hpp
    Common.hpp
    CharacterController.cpp
    CharacterController.hpp
    CollisionObject.cpp
    CollisionObject.hpp
    CollisionShape.cpp
    CollisionShape.hpp
    ConstraintSolver.cpp
    ConstraintSolver.hpp
    Engine.cpp
    Engine.hpp
    Icd.cpp
    MotionState.cpp
    MotionState.hpp
    World.cpp
    World.hpp
)

include_directories(bullet3/src)
add_definitions(-DAPHY_BUILD)

add_library(APhyBullet SHARED ${APHY_Bullet_SOURCES}
    $<TARGET_OBJECTS:LinearMath> $<TARGET_OBJECTS:BulletCollision> $<TARGET_OBJECTS:BulletDynamics> $<TARGET_OBJECTS:BulletSoftBody>)
target_link_libraries(APhyBullet)
