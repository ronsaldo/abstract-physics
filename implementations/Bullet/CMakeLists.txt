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
    broadphase.cpp
    broadphase.hpp
    collision_configuration.cpp
    collision_configuration.hpp
    collision_dispatcher.cpp
    collision_dispatcher.hpp
    common.hpp
    collision_object.cpp
    collision_object.hpp
    collision_shape.cpp
    collision_shape.hpp
    constraint_solver.cpp
    constraint_solver.hpp
    engine.cpp
    engine.hpp
    icd.c
    object.hpp
    world.cpp
    world.hpp
)

include_directories(bullet3/src)
add_definitions(-DAPHY_BUILD)

add_library(APhyBullet SHARED ${APHY_Bullet_SOURCES}
    $<TARGET_OBJECTS:LinearMath> $<TARGET_OBJECTS:BulletCollision> $<TARGET_OBJECTS:BulletDynamics> $<TARGET_OBJECTS:BulletSoftBody>)
target_link_libraries(APhyBullet)
